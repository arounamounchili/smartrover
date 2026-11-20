//! Wi-Fi manager (Station & SoftAP modes), HTTP/1.1 keep-alive server,
//! telemetry JSON API, and session-preemptible MJPEG streamer.

mod dhcp;
mod http;

#[cfg(target_arch = "xtensa")]
mod target_impl {
    use super::dhcp::DhcpServer;
    use super::http::{HttpRoute, MAX_HTTP_HEADER_BYTES, ParseProgress, parse_http_request};
    use crate::adapters::SharedRoverState;
    use crate::config::network::{
        AP_DHCP_POOL_START_OCTET, AP_IP_OCTETS, RSSI_POLL_INTERVAL_MS, STA_RETRY_DELAY_MS,
    };
    use core::fmt::Write as _;
    use core::sync::atomic::{AtomicU32, Ordering};
    use embassy_futures::select::{Either, select};
    use embassy_net::Ipv4Address;
    use embassy_net::udp::{PacketMetadata, UdpSocket};
    use embassy_time::{Duration, Timer};
    use embedded_io_async::Write as _;
    use esp_radio::wifi::{Interface as WifiDevice, WifiController};
    use log::{error, info, warn};
    use serde::Serialize;

    /// Active Wi-Fi operating mode passed to [`connection_task`].
    #[derive(Clone, Copy)]
    pub enum WifiModeInfo {
        Station { ssid: &'static str },
        SoftAp { ssid: &'static str },
    }

    #[embassy_executor::task]
    pub async fn net_task(
        stack: embassy_net::Stack<'static>,
        mut runner: embassy_net::Runner<'static, WifiDevice<'static>>,
    ) -> ! {
        embassy_futures::join::join(runner.run(), async {
            loop {
                stack.wait_config_up().await;
                if let Some(config) = stack.config_v4() {
                    info!(
                        "Network up! Rover Cockpit: http://{}/",
                        config.address.address()
                    );
                }
                stack.wait_config_down().await;
                info!("Network link down. Waiting for reconnection...");
            }
        })
        .await;
        unreachable!()
    }

    #[embassy_executor::task]
    pub async fn connection_task(
        mut controller: WifiController<'static>,
        mode: WifiModeInfo,
        state: &'static SharedRoverState,
    ) {
        info!("Wi-Fi connection task started");

        match mode {
            WifiModeInfo::SoftAp { ssid } => {
                state.publish_rssi(0);
                info!(
                    "SoftAP Active! SSID='{}' Cockpit=http://{}.{}.{}.{}/",
                    ssid, AP_IP_OCTETS[0], AP_IP_OCTETS[1], AP_IP_OCTETS[2], AP_IP_OCTETS[3]
                );
                // Keep `controller` alive so the SoftAP stays running.
                core::future::pending::<()>().await;
            }
            WifiModeInfo::Station { ssid } => loop {
                info!("Connecting to {ssid}...");
                match controller.connect_async().await {
                    Ok(_) => {
                        info!("Wi-Fi Connected!");
                        loop {
                            if let Ok(rssi) = controller.rssi() {
                                state.publish_rssi(rssi.clamp(-127, 0) as i8);
                            }
                            if let Either::First(_) = select(
                                controller.wait_for_disconnect_async(),
                                Timer::after(Duration::from_millis(RSSI_POLL_INTERVAL_MS)),
                            )
                            .await
                            {
                                break;
                            }
                        }
                        state.publish_rssi(0);
                        warn!("Wi-Fi Disconnected, retrying...");
                    }
                    Err(e) => {
                        state.publish_rssi(0);
                        error!("Connection failed: {e:?}");
                    }
                }
                Timer::after(Duration::from_millis(STA_RETRY_DELAY_MS)).await;
            },
        }
    }

    #[embassy_executor::task]
    pub async fn dhcp_task(stack: embassy_net::Stack<'static>) {
        stack.wait_config_up().await;

        let mut rx_meta = [PacketMetadata::EMPTY; 2];
        let mut tx_meta = [PacketMetadata::EMPTY; 2];
        let mut rx_buf = [0u8; 576];
        let mut tx_buf = [0u8; 576];

        let mut socket =
            UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
        if let Err(e) = socket.bind(67) {
            error!("SoftAP DHCP bind(67) failed: {e:?}");
            return;
        }
        info!("SoftAP DHCPv4 server listening on UDP/67");

        let mut server = DhcpServer::new(AP_IP_OCTETS, AP_DHCP_POOL_START_OCTET);
        let mut req_pkt = [0u8; 576];
        let mut rep_pkt = [0u8; 548];

        loop {
            match socket.recv_from(&mut req_pkt).await {
                Ok((n, _meta)) => {
                    if let Some(reply_len) = server.handle_packet(&req_pkt[..n], &mut rep_pkt)
                        && let Err(e) = socket
                            .send_to(&rep_pkt[..reply_len], (Ipv4Address::BROADCAST, 68))
                            .await
                    {
                        warn!("DHCP send error: {e:?}");
                    }
                }
                Err(e) => {
                    warn!("DHCP recv error: {e:?}");
                    Timer::after(Duration::from_millis(50)).await;
                }
            }
        }
    }

    const DASHBOARD_GZ: &[u8] = include_bytes!("../../dashboard/dashboard.html.gz");

    #[derive(Serialize)]
    struct StatusResponse {
        dist: u16,
        batt: i32,
        state: u8,
        adas_active: bool,
        left_pwm: i32,
        right_pwm: i32,
        rssi: i8,
        tof_ok: bool,
        imu_ok: bool,
    }

    /// Context for each concurrent HTTP server worker task.
    #[derive(Clone, Copy)]
    pub struct WebContext {
        pub state: &'static SharedRoverState,
        pub frame: &'static crate::camera::SharedFrame,
        pub stream_session: &'static AtomicU32,
    }

    /// Pre-allocated per-worker socket and HTTP header buffers.
    #[repr(align(4))]
    pub struct WebBuffers {
        rx: [u8; 1024],
        tx: [u8; 2048],
        read: [u8; MAX_HTTP_HEADER_BYTES],
    }

    impl Default for WebBuffers {
        fn default() -> Self {
            Self::new()
        }
    }

    impl WebBuffers {
        pub const fn new() -> Self {
            Self {
                rx: [0u8; 1024],
                tx: [0u8; 2048],
                read: [0u8; MAX_HTTP_HEADER_BYTES],
            }
        }
    }

    fn build_status_response(state: &SharedRoverState) -> StatusResponse {
        let snap = state.read_telemetry();
        StatusResponse {
            dist: snap.distance_mm,
            batt: snap.battery_pct,
            state: snap.mode as u8,
            adas_active: snap.adas_intervening,
            left_pwm: snap.left_duty,
            right_pwm: snap.right_duty,
            rssi: state.rssi_dbm(),
            tof_ok: snap.tof_healthy,
            imu_ok: snap.imu_healthy,
        }
    }

    const _: () = assert!(crate::config::WEB_WORKER_COUNT == 3);

    #[embassy_executor::task(pool_size = 3)]
    pub async fn web_task(
        stack: embassy_net::Stack<'static>,
        ctx: WebContext,
        buffers: &'static mut WebBuffers,
    ) {
        use embassy_net::tcp::TcpSocket;

        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!(
                "Web Cockpit worker ready on http://{}/",
                config.address.address()
            );
        }

        loop {
            let mut socket = TcpSocket::new(stack, &mut buffers.rx, &mut buffers.tx);
            socket.set_timeout(Some(Duration::from_millis(1000)));
            socket.set_keep_alive(Some(Duration::from_secs(2)));

            if socket.accept(crate::config::WEB_PORT).await.is_err() {
                continue;
            }

            let mut buf_len: usize = 0;

            'conn: loop {
                let parsed = loop {
                    match parse_http_request(&buffers.read[..buf_len]) {
                        ParseProgress::Complete { request, consumed } => {
                            buffers.read.copy_within(consumed..buf_len, 0);
                            buf_len -= consumed;
                            break Some(request);
                        }
                        ParseProgress::HeaderTooLarge => {
                            let resp = "HTTP/1.1 431 Request Header Fields Too Large\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
                            let _ = socket.write_all(resp.as_bytes()).await;
                            break None;
                        }
                        ParseProgress::BadRequest => {
                            let resp = "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nContent-Length: 11\r\nConnection: close\r\n\r\nBad Request";
                            let _ = socket.write_all(resp.as_bytes()).await;
                            break None;
                        }
                        ParseProgress::Incomplete => {
                            match socket.read(&mut buffers.read[buf_len..]).await {
                                Ok(0) | Err(_) => break None,
                                Ok(n) => buf_len += n,
                            }
                        }
                    }
                };

                let Some(request) = parsed else {
                    break 'conn;
                };

                let conn_hdr = if request.keep_alive {
                    "keep-alive"
                } else {
                    "close"
                };

                match request.route {
                    HttpRoute::Options => {
                        let resp = if request.keep_alive {
                            "HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nConnection: keep-alive\r\n\r\n"
                        } else {
                            "HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nConnection: close\r\n\r\n"
                        };
                        if socket.write_all(resp.as_bytes()).await.is_err() {
                            break 'conn;
                        }
                    }
                    HttpRoute::Control(cmd) => {
                        ctx.state.publish_command(cmd);
                        if !write_status_json(&mut socket, ctx.state, conn_hdr).await {
                            break 'conn;
                        }
                    }
                    HttpRoute::Status => {
                        if !write_status_json(&mut socket, ctx.state, conn_hdr).await {
                            break 'conn;
                        }
                    }
                    HttpRoute::Index => {
                        let mut header: heapless::String<256> = heapless::String::new();
                        let _ = write!(
                            &mut header,
                            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Encoding: gzip\r\nCache-Control: no-cache\r\nContent-Length: {}\r\nConnection: {}\r\n\r\n",
                            DASHBOARD_GZ.len(),
                            conn_hdr
                        );
                        if socket.write_all(header.as_bytes()).await.is_err()
                            || socket.write_all(DASHBOARD_GZ).await.is_err()
                        {
                            break 'conn;
                        }
                    }
                    HttpRoute::Stream => {
                        serve_mjpeg_stream(&mut socket, &ctx).await;
                        break 'conn;
                    }
                    HttpRoute::NotFound => {
                        let resp = if request.keep_alive {
                            "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: 9\r\nConnection: keep-alive\r\n\r\nNot Found"
                        } else {
                            "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: 9\r\nConnection: close\r\n\r\nNot Found"
                        };
                        if socket.write_all(resp.as_bytes()).await.is_err() {
                            break 'conn;
                        }
                    }
                }

                if !request.keep_alive {
                    break 'conn;
                }
            }

            socket.close();
            let _ = socket.flush().await;
        }
    }

    async fn write_status_json(
        socket: &mut embassy_net::tcp::TcpSocket<'_>,
        state: &SharedRoverState,
        conn_hdr: &str,
    ) -> bool {
        let status = build_status_response(state);
        let Ok(body) = serde_json_core::to_string::<StatusResponse, 256>(&status) else {
            return false;
        };

        let mut header: heapless::String<256> = heapless::String::new();
        let _ = write!(
            &mut header,
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nContent-Length: {}\r\nConnection: {}\r\n\r\n",
            body.len(),
            conn_hdr
        );

        socket.write_all(header.as_bytes()).await.is_ok()
            && socket.write_all(body.as_bytes()).await.is_ok()
    }

    async fn serve_mjpeg_stream(socket: &mut embassy_net::tcp::TcpSocket<'_>, ctx: &WebContext) {
        socket.set_timeout(Some(Duration::from_millis(2000)));
        let header = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\nPragma: no-cache\r\nConnection: close\r\n\r\n";
        if socket.write_all(header.as_bytes()).await.is_err() {
            return;
        }

        let current_session = ctx
            .stream_session
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1);
        // Wake any previous `/stream` session immediately so it observes preemption and releases its socket.
        ctx.frame.notify_stream_preempt();
        // Record the last published version before waking Core 1 so fresh frames are never skipped.
        let mut last_version = ctx.frame.version();
        let _viewer_guard = ctx.frame.acquire_viewer();

        loop {
            if ctx.stream_session.load(Ordering::Acquire) != current_session {
                info!("Stream session preempted; releasing socket");
                break;
            }

            let current_version = ctx.frame.version();
            if current_version == last_version {
                select(
                    ctx.frame.wait_frame_signal(),
                    Timer::after(Duration::from_millis(100)),
                )
                .await;
                continue;
            }
            last_version = current_version;

            let frame_guard = ctx.frame.lock_active_frame().await;
            if frame_guard.is_empty() {
                continue;
            }

            let mut part_header: heapless::String<128> = heapless::String::new();
            let _ = write!(
                &mut part_header,
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                frame_guard.len()
            );

            if socket.write_all(part_header.as_bytes()).await.is_err() {
                break;
            }

            let mut failed = false;
            for chunk in frame_guard.chunks(1024) {
                if ctx.stream_session.load(Ordering::Relaxed) != current_session
                    || socket.write_all(chunk).await.is_err()
                {
                    failed = true;
                    break;
                }
            }

            if failed || socket.write_all(b"\r\n").await.is_err() {
                break;
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
pub use target_impl::{
    WebBuffers, WebContext, WifiModeInfo, connection_task, dhcp_task, net_task, web_task,
};
