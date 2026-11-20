//! Minimal `no_std` RFC 2131 DHCPv4 server responder for SoftAP mode.

const DHCP_MAGIC_COOKIE: [u8; 4] = [99, 130, 83, 99];
const DHCP_DISCOVER: u8 = 1;
const DHCP_OFFER: u8 = 2;
const DHCP_REQUEST: u8 = 3;
const DHCP_ACK: u8 = 5;

const MAX_LEASES: usize = 8;

#[derive(Clone, Copy)]
struct LeaseEntry {
    mac: [u8; 6],
    last_octet: u8,
    in_use: bool,
}

impl LeaseEntry {
    const fn empty() -> Self {
        Self {
            mac: [0; 6],
            last_octet: 0,
            in_use: false,
        }
    }
}

/// Small static-memory DHCPv4 lease allocator for SoftAP clients.
pub(super) struct DhcpServer {
    server_ip: [u8; 4],
    pool_start_octet: u8,
    leases: [LeaseEntry; MAX_LEASES],
}

impl DhcpServer {
    pub(super) const fn new(server_ip: [u8; 4], pool_start_octet: u8) -> Self {
        Self {
            server_ip,
            pool_start_octet,
            leases: [LeaseEntry::empty(); MAX_LEASES],
        }
    }

    fn allocate_ip(&mut self, mac: [u8; 6]) -> [u8; 4] {
        for entry in &self.leases {
            if entry.in_use && entry.mac == mac {
                return [
                    self.server_ip[0],
                    self.server_ip[1],
                    self.server_ip[2],
                    entry.last_octet,
                ];
            }
        }

        for (idx, entry) in self.leases.iter_mut().enumerate() {
            if !entry.in_use {
                let octet = self.pool_start_octet.saturating_add(idx as u8).min(254);
                *entry = LeaseEntry {
                    mac,
                    last_octet: octet,
                    in_use: true,
                };
                return [
                    self.server_ip[0],
                    self.server_ip[1],
                    self.server_ip[2],
                    octet,
                ];
            }
        }

        // Table full: assign deterministic host octet below broadcast (.255)
        let octet = self
            .pool_start_octet
            .saturating_add(MAX_LEASES as u8 + (mac[5] % 64))
            .min(254);
        [
            self.server_ip[0],
            self.server_ip[1],
            self.server_ip[2],
            octet,
        ]
    }

    /// Processes a raw UDP payload received on port 67 and writes a DHCP reply into `out`.
    /// Returns `Some(len)` when a reply should be broadcast to `255.255.255.255:68`.
    pub(super) fn handle_packet(&mut self, packet: &[u8], out: &mut [u8; 548]) -> Option<usize> {
        if packet.len() < 240 {
            return None;
        }
        // BOOTREQUEST (1), Ethernet (1), hlen (6)
        if packet[0] != 1 || packet[1] != 1 || packet[2] != 6 {
            return None;
        }
        if packet[236..240] != DHCP_MAGIC_COOKIE {
            return None;
        }

        let msg_type = parse_message_type(&packet[240..])?;
        let reply_type = match msg_type {
            DHCP_DISCOVER => DHCP_OFFER,
            DHCP_REQUEST => DHCP_ACK,
            _ => return None,
        };

        let mut mac = [0u8; 6];
        mac.copy_from_slice(&packet[28..34]);
        let client_ip = self.allocate_ip(mac);

        out.fill(0);
        out[0] = 2; // BOOTREPLY
        out[1] = 1; // Ethernet
        out[2] = 6; // HLEN
        out[4..8].copy_from_slice(&packet[4..8]); // XID
        // Set broadcast flag so clients without configured IP accept OFFER/ACK cleanly
        out[10] = packet[10] | 0x80;
        out[11] = packet[11];
        out[16..20].copy_from_slice(&client_ip); // YIADDR
        out[20..24].copy_from_slice(&self.server_ip); // SIADDR
        out[28..34].copy_from_slice(&mac); // CHADDR
        out[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE);

        let mut cursor = 240;
        // Option 53: DHCP Message Type
        out[cursor..cursor + 3].copy_from_slice(&[53, 1, reply_type]);
        cursor += 3;
        // Option 54: Server Identifier
        out[cursor..cursor + 2].copy_from_slice(&[54, 4]);
        out[cursor + 2..cursor + 6].copy_from_slice(&self.server_ip);
        cursor += 6;
        // Option 51: IP Address Lease Time (3600s)
        out[cursor..cursor + 6].copy_from_slice(&[51, 4, 0, 0, 0x0E, 0x10]);
        cursor += 6;
        // Option 1: Subnet Mask (255.255.255.0)
        out[cursor..cursor + 6].copy_from_slice(&[1, 4, 255, 255, 255, 0]);
        cursor += 6;
        // Option 3: Router
        out[cursor..cursor + 2].copy_from_slice(&[3, 4]);
        out[cursor + 2..cursor + 6].copy_from_slice(&self.server_ip);
        cursor += 6;
        // Option 6: DNS Server
        out[cursor..cursor + 2].copy_from_slice(&[6, 4]);
        out[cursor + 2..cursor + 6].copy_from_slice(&self.server_ip);
        cursor += 6;
        // Option 255: End
        out[cursor] = 255;
        cursor += 1;

        Some(cursor.max(300))
    }
}

fn parse_message_type(options: &[u8]) -> Option<u8> {
    let mut i = 0;
    while i < options.len() {
        let code = options[i];
        if code == 255 {
            break;
        }
        if code == 0 {
            i += 1;
            continue;
        }
        if i + 1 >= options.len() {
            break;
        }
        let len = options[i + 1] as usize;
        if i + 2 + len > options.len() {
            break;
        }
        if code == 53 && len == 1 {
            return Some(options[i + 2]);
        }
        i += 2 + len;
    }
    None
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    fn build_discover(mac: [u8; 6], msg_type: u8) -> [u8; 300] {
        let mut pkt = [0u8; 300];
        pkt[0] = 1;
        pkt[1] = 1;
        pkt[2] = 6;
        pkt[4..8].copy_from_slice(&[0x12, 0x34, 0x56, 0x78]);
        pkt[28..34].copy_from_slice(&mac);
        pkt[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE);
        pkt[240..244].copy_from_slice(&[53, 1, msg_type, 255]);
        pkt
    }

    #[test]
    fn test_dhcp_discover_and_request_lease_stability() {
        let mut server = DhcpServer::new([192, 168, 4, 1], 100);
        let mac_a = [0xAA, 0xBB, 0xCC, 0x01, 0x02, 0x03];
        let mac_b = [0xAA, 0xBB, 0xCC, 0x01, 0x02, 0x04];

        let mut out = [0u8; 548];
        let discover = build_discover(mac_a, DHCP_DISCOVER);
        let len = server.handle_packet(&discover, &mut out).unwrap();
        assert!(len >= 300);
        assert_eq!(out[0], 2);
        assert_eq!(&out[16..20], &[192, 168, 4, 100]);
        assert_eq!(parse_message_type(&out[240..len]), Some(DHCP_OFFER));

        let request = build_discover(mac_a, DHCP_REQUEST);
        let len2 = server.handle_packet(&request, &mut out).unwrap();
        assert_eq!(&out[16..20], &[192, 168, 4, 100]);
        assert_eq!(parse_message_type(&out[240..len2]), Some(DHCP_ACK));

        let discover_b = build_discover(mac_b, DHCP_DISCOVER);
        let _ = server.handle_packet(&discover_b, &mut out).unwrap();
        assert_eq!(&out[16..20], &[192, 168, 4, 101]);
    }

    #[test]
    fn test_dhcp_rejects_short_and_invalid_header_packets() {
        let mut server = DhcpServer::new([192, 168, 4, 1], 100);
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
        let mut out = [0xAAu8; 548];

        // Empty and 239-byte packets (< 240 bytes minimum).
        assert_eq!(server.handle_packet(&[], &mut out), None);
        let short = [0u8; 239];
        assert_eq!(server.handle_packet(&short, &mut out), None);

        // Invalid BOOTREQUEST op (e.g. BOOTREPLY = 2).
        let mut bad_op = build_discover(mac, DHCP_DISCOVER);
        bad_op[0] = 2;
        assert_eq!(server.handle_packet(&bad_op, &mut out), None);

        // Invalid hardware type (htype != 1).
        let mut bad_htype = build_discover(mac, DHCP_DISCOVER);
        bad_htype[1] = 6;
        assert_eq!(server.handle_packet(&bad_htype, &mut out), None);

        // Invalid hardware address length (hlen != 6).
        let mut bad_hlen = build_discover(mac, DHCP_DISCOVER);
        bad_hlen[2] = 4;
        assert_eq!(server.handle_packet(&bad_hlen, &mut out), None);

        // Invalid DHCP magic cookie.
        let mut bad_cookie = build_discover(mac, DHCP_DISCOVER);
        bad_cookie[236..240].copy_from_slice(&[99, 130, 83, 0]);
        assert_eq!(server.handle_packet(&bad_cookie, &mut out), None);

        // Unsupported DHCP message types, including DHCPRELEASE (7) and DHCPINFORM (8).
        let release = build_discover(mac, 7);
        assert_eq!(server.handle_packet(&release, &mut out), None);
        let inform = build_discover(mac, 8);
        assert_eq!(server.handle_packet(&inform, &mut out), None);

        // Output buffer must remain untouched when all packets are rejected.
        assert!(out.iter().all(|&b| b == 0xAA));
    }

    #[test]
    fn test_dhcp_option_parsing_edge_cases() {
        let mut server = DhcpServer::new([192, 168, 4, 1], 100);
        let mac = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60];
        let mut out = [0u8; 548];

        // Helper for building a packet with custom options slice.
        let make_pkt = |opts: &[u8]| -> ([u8; 260], usize) {
            let mut pkt = [0u8; 260];
            pkt[0] = 1;
            pkt[1] = 1;
            pkt[2] = 6;
            pkt[28..34].copy_from_slice(&mac);
            pkt[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE);
            let total = 240 + opts.len();
            pkt[240..total].copy_from_slice(opts);
            (pkt, total)
        };

        // Truncated option header (code present, length byte missing).
        let (pkt, n) = make_pkt(&[50]);
        assert_eq!(server.handle_packet(&pkt[..n], &mut out), None);

        // Option declaring length larger than remaining packet bytes.
        let (pkt, n) = make_pkt(&[50, 10, 192, 168]);
        assert_eq!(server.handle_packet(&pkt[..n], &mut out), None);

        // Option 53 with wrong length (!= 1) is ignored; no valid msg type -> None.
        let (pkt, n) = make_pkt(&[53, 2, DHCP_DISCOVER, 0, 255]);
        assert_eq!(server.handle_packet(&pkt[..n], &mut out), None);

        // Missing end option (255) without option 53 -> None.
        let (pkt, n) = make_pkt(&[12, 3, b'a', b'b', b'c']);
        assert_eq!(server.handle_packet(&pkt[..n], &mut out), None);

        // Missing end option (255) after option 53 -> still parses option 53 cleanly.
        let (pkt, n) = make_pkt(&[53, 1, DHCP_DISCOVER]);
        assert!(server.handle_packet(&pkt[..n], &mut out).is_some());

        // Single and multiple Option 0 padding + unexpected option ordering before option 53.
        let (pkt, n) = make_pkt(&[
            0,
            0,
            0,
            50,
            4,
            192,
            168,
            4,
            100,
            0,
            55,
            2,
            1,
            3,
            0,
            53,
            1,
            DHCP_REQUEST,
            255,
        ]);
        let len = server.handle_packet(&pkt[..n], &mut out).unwrap();
        assert_eq!(parse_message_type(&out[240..len]), Some(DHCP_ACK));

        // End option (255) before option 53 terminates option parsing early -> None.
        let (pkt, n) = make_pkt(&[255, 53, 1, DHCP_DISCOVER]);
        assert_eq!(server.handle_packet(&pkt[..n], &mut out), None);
    }

    #[test]
    fn test_dhcp_lease_pool_exhaustion_and_state_isolation() {
        let mut server = DhcpServer::new([192, 168, 4, 1], 100);
        let mut out = [0u8; 548];

        // Fill all MAX_LEASES (8) slots with distinct MACs.
        for i in 0..MAX_LEASES {
            let mac = [0xAA, 0x00, 0x00, 0x00, 0x00, i as u8];
            let discover = build_discover(mac, DHCP_DISCOVER);
            let len = server.handle_packet(&discover, &mut out).unwrap();
            assert_eq!(out[16..20], [192, 168, 4, 100 + i as u8]);
            assert_eq!(parse_message_type(&out[240..len]), Some(DHCP_OFFER));

            // Repeated DISCOVER and REQUEST for the same MAC keeps the same IP.
            let discover_again = build_discover(mac, DHCP_DISCOVER);
            let _ = server.handle_packet(&discover_again, &mut out).unwrap();
            assert_eq!(out[16..20], [192, 168, 4, 100 + i as u8]);

            let request = build_discover(mac, DHCP_REQUEST);
            let len_ack = server.handle_packet(&request, &mut out).unwrap();
            assert_eq!(out[16..20], [192, 168, 4, 100 + i as u8]);
            assert_eq!(parse_message_type(&out[240..len_ack]), Some(DHCP_ACK));
        }

        // Send malformed packets; verify existing leases are uncorrupted.
        let mut invalid = build_discover([0xAA, 0x00, 0x00, 0x00, 0x00, 0], DHCP_DISCOVER);
        invalid[236] = 0; // corrupt magic cookie
        assert_eq!(server.handle_packet(&invalid, &mut out), None);

        for i in 0..MAX_LEASES {
            let mac = [0xAA, 0x00, 0x00, 0x00, 0x00, i as u8];
            let req = build_discover(mac, DHCP_REQUEST);
            assert!(server.handle_packet(&req, &mut out).is_some());
            assert_eq!(out[16..20], [192, 168, 4, 100 + i as u8]);
        }

        // 9th and 10th distinct MACs (> MAX_LEASES) hit pool exhaustion fallback:
        // pool_start_octet + MAX_LEASES + (mac[5] % 64).
        let mac_9 = [0xBB, 0x00, 0x00, 0x00, 0x00, 10];
        let _ = server
            .handle_packet(&build_discover(mac_9, DHCP_DISCOVER), &mut out)
            .unwrap();
        assert_eq!(out[16..20], [192, 168, 4, 100 + MAX_LEASES as u8 + 10]);

        let mac_10 = [0xBB, 0x00, 0x00, 0x00, 0x00, 70]; // 70 % 64 == 6
        let _ = server
            .handle_packet(&build_discover(mac_10, DHCP_REQUEST), &mut out)
            .unwrap();
        assert_eq!(out[16..20], [192, 168, 4, 100 + MAX_LEASES as u8 + 6]);

        // High pool_start_octet must never hand out broadcast address (.255).
        let mut high_pool = DhcpServer::new([192, 168, 4, 1], 250);
        for i in 0..=MAX_LEASES {
            let mac = [0xCC, 0x00, 0x00, 0x00, 0x00, i as u8];
            let _ = high_pool
                .handle_packet(&build_discover(mac, DHCP_DISCOVER), &mut out)
                .unwrap();
            assert!(out[19] <= 254, "allocated broadcast octet: {}", out[19]);
        }
    }

    #[test]
    fn test_dhcp_dirty_output_buffer_is_zeroed_and_length_exact() {
        let mut server = DhcpServer::new([192, 168, 4, 1], 100);
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let discover = build_discover(mac, DHCP_DISCOVER);

        let mut out = [0xFFu8; 548];
        let len = server.handle_packet(&discover, &mut out).unwrap();

        // Standard DHCP reply writes 274 bytes and pads to minimum 300 bytes.
        assert_eq!(len, 300);
        assert_eq!(out[0], 2); // BOOTREPLY
        assert_eq!(&out[4..8], &[0x12, 0x34, 0x56, 0x78]); // xid echoed
        assert_eq!(&out[16..20], &[192, 168, 4, 100]); // yiaddr
        assert_eq!(&out[20..24], &[192, 168, 4, 1]); // siaddr
        assert_eq!(&out[28..34], &mac); // chaddr
        assert_eq!(&out[236..240], &DHCP_MAGIC_COOKIE);
        // Padding between end option and 300, as well as remainder of out, must be zeroed.
        assert!(out[274..548].iter().all(|&b| b == 0));
    }
}
