//! Pure `no_std` HTTP/1.x request parser and route matcher for SmartRover.

use crate::domain::ports::{DriveCommand, SpeedGear};

/// Maximum request header size accepted on port 80.
pub(super) const MAX_HTTP_HEADER_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HttpMethod {
    Get,
    Post,
    Options,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum HttpRoute {
    Index,
    Control(DriveCommand),
    Status,
    Stream,
    Options,
    NotFound,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ParsedHttpRequest {
    pub(super) route: HttpRoute,
    pub(super) keep_alive: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum ParseProgress {
    Complete {
        request: ParsedHttpRequest,
        consumed: usize,
    },
    Incomplete,
    HeaderTooLarge,
    BadRequest,
}

/// Parses an HTTP request header block from `buf`.
///
/// Returns `Complete` once the blank header terminator (`\r\n\r\n` or `\n\n`)
/// has arrived, `Incomplete` when more bytes are needed, `HeaderTooLarge` when
/// no terminator appears within `MAX_HTTP_HEADER_BYTES`, or `BadRequest` on
/// malformed headers.
pub(super) fn parse_http_request(buf: &[u8]) -> ParseProgress {
    let Some((header_end, consumed)) = find_header_end(buf) else {
        return if buf.len() >= MAX_HTTP_HEADER_BYTES {
            ParseProgress::HeaderTooLarge
        } else {
            ParseProgress::Incomplete
        };
    };

    let Ok(header_text) = core::str::from_utf8(&buf[..header_end]) else {
        return ParseProgress::BadRequest;
    };

    let mut lines = header_text.split('\n').map(|l| l.trim_end_matches('\r'));
    let request_line = lines.next().unwrap_or("");

    let mut parts = request_line.split_whitespace();
    let Some(method_str) = parts.next() else {
        return ParseProgress::BadRequest;
    };
    let Some(target) = parts.next() else {
        return ParseProgress::BadRequest;
    };
    let Some(version) = parts.next() else {
        return ParseProgress::BadRequest;
    };
    if !version.starts_with("HTTP/1.") || parts.next().is_some() {
        return ParseProgress::BadRequest;
    }

    let method = match method_str {
        "GET" => HttpMethod::Get,
        "POST" => HttpMethod::Post,
        "OPTIONS" => HttpMethod::Options,
        _ => HttpMethod::Other,
    };

    let (path, query) = target.split_once('?').unwrap_or((target, ""));

    let mut keep_alive = version == "HTTP/1.1";
    let mut has_body = false;
    for line in lines {
        if let Some((name, val)) = line.split_once(':') {
            let hdr = name.trim();
            let v = val.trim();
            if hdr.eq_ignore_ascii_case("connection") {
                if v.eq_ignore_ascii_case("close") {
                    keep_alive = false;
                } else if v.eq_ignore_ascii_case("keep-alive") {
                    keep_alive = true;
                }
            } else if (hdr.eq_ignore_ascii_case("content-length")
                && v.parse::<usize>().unwrap_or(1) > 0)
                || hdr.eq_ignore_ascii_case("transfer-encoding")
            {
                // Routes on this server do not drain request bodies, so close after replying.
                has_body = true;
            }
        }
    }
    if has_body {
        keep_alive = false;
    }

    let route = match (method, path) {
        (HttpMethod::Options, _) => HttpRoute::Options,
        (HttpMethod::Get, "/" | "/index.html") => HttpRoute::Index,
        (HttpMethod::Get, "/status") => HttpRoute::Status,
        (HttpMethod::Post, "/control") => HttpRoute::Control(parse_control_query(query)),
        (HttpMethod::Get, "/stream") => {
            keep_alive = false;
            HttpRoute::Stream
        }
        _ => HttpRoute::NotFound,
    };

    ParseProgress::Complete {
        request: ParsedHttpRequest { route, keep_alive },
        consumed,
    }
}

/// Parses a `t=<milli_throttle>&s=<milli_steer>&g=<gear>&b=<brake>&a=<adas>` query string.
fn parse_control_query(query: &str) -> DriveCommand {
    let mut throttle_milli: i32 = 0;
    let mut steering_milli: i32 = 0;
    let mut gear = SpeedGear::Normal;
    let mut brake = false;
    let mut adas_enabled = true;

    for pair in query.split('&') {
        if let Some((k, v)) = pair.split_once('=') {
            match k {
                "t" => throttle_milli = v.parse::<i32>().unwrap_or(0).clamp(-1000, 1000),
                "s" => steering_milli = v.parse::<i32>().unwrap_or(0).clamp(-1000, 1000),
                "g" => gear = SpeedGear::from_u8(v.parse::<u8>().unwrap_or(2)),
                "b" => brake = v == "1" || v == "true",
                "a" => adas_enabled = v != "0" && v != "false",
                _ => {}
            }
        }
    }

    DriveCommand {
        throttle: (throttle_milli as f32) / 1000.0,
        steering: (steering_milli as f32) / 1000.0,
        gear,
        brake,
        adas_enabled,
    }
}

fn find_header_end(buf: &[u8]) -> Option<(usize, usize)> {
    let mut i = 0;
    while i + 1 < buf.len() {
        if i + 3 < buf.len() && &buf[i..i + 4] == b"\r\n\r\n" {
            return Some((i, i + 4));
        }
        if &buf[i..i + 2] == b"\n\n" {
            return Some((i, i + 2));
        }
        i += 1;
    }
    None
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_http_control_query_parser() {
        let query = "t=850&s=-400&g=3&b=1&a=0&unknown=1";
        let cmd = parse_control_query(query);
        assert!((cmd.throttle - 0.85).abs() < 1e-3);
        assert!((cmd.steering + 0.40).abs() < 1e-3);
        assert_eq!(cmd.gear, SpeedGear::Sport);
        assert!(cmd.brake);
        assert!(!cmd.adas_enabled);

        let clamped = parse_control_query("t=5000&s=-9999");
        assert_eq!(clamped.throttle, 1.0);
        assert_eq!(clamped.steering, -1.0);
    }

    #[test]
    fn test_http_routes_and_keep_alive() {
        let req = b"GET / HTTP/1.1\r\nHost: rover.local\r\n\r\n";
        assert_eq!(
            parse_http_request(req),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Index,
                    keep_alive: true,
                },
                consumed: req.len(),
            }
        );

        let ctrl = b"POST /control?t=500&s=-250&g=3&b=1&a=0 HTTP/1.1\r\nConnection: close\r\n\r\n";
        match parse_http_request(ctrl) {
            ParseProgress::Complete { request, consumed } => {
                assert_eq!(consumed, ctrl.len());
                assert!(!request.keep_alive);
                assert_eq!(
                    request.route,
                    HttpRoute::Control(DriveCommand {
                        throttle: 0.5,
                        steering: -0.25,
                        gear: SpeedGear::Sport,
                        brake: true,
                        adas_enabled: false,
                    })
                );
            }
            other => panic!("unexpected parse result: {:?}", other),
        }

        let stream = b"GET /stream?t=12345 HTTP/1.1\r\nConnection: keep-alive\r\n\r\n";
        assert_eq!(
            parse_http_request(stream),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Stream,
                    keep_alive: false,
                },
                consumed: stream.len(),
            }
        );

        let spoofed = b"GET /foo/control?t=1000 HTTP/1.1\r\n\r\n";
        assert_eq!(
            parse_http_request(spoofed),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::NotFound,
                    keep_alive: true,
                },
                consumed: spoofed.len(),
            }
        );

        // /control only accepts POST so state mutations cannot be triggered via GET.
        let get_ctrl = b"GET /control?t=1000 HTTP/1.1\r\n\r\n";
        assert_eq!(
            parse_http_request(get_ctrl),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::NotFound,
                    keep_alive: true,
                },
                consumed: get_ctrl.len(),
            }
        );

        let with_body = b"POST /control?t=0 HTTP/1.1\r\nContent-Length: 4\r\n\r\n";
        assert_eq!(
            parse_http_request(with_body),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Control(DriveCommand::default()),
                    keep_alive: false,
                },
                consumed: with_body.len(),
            }
        );
    }

    #[test]
    fn test_http_partial_and_invalid_headers() {
        assert_eq!(
            parse_http_request(b"GET /status HTTP/1.1\r\nHost: x"),
            ParseProgress::Incomplete
        );
        assert_eq!(
            parse_http_request(b"BADREQUEST\r\n\r\n"),
            ParseProgress::BadRequest
        );
        let oversized = [b'A'; MAX_HTTP_HEADER_BYTES];
        assert_eq!(
            parse_http_request(&oversized),
            ParseProgress::HeaderTooLarge
        );
    }

    #[test]
    fn test_http_all_methods_and_routes() {
        let index_html = b"GET /index.html HTTP/1.1\r\n\r\n";
        assert_eq!(
            parse_http_request(index_html),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Index,
                    keep_alive: true,
                },
                consumed: index_html.len(),
            }
        );

        let status = b"GET /status HTTP/1.1\r\n\r\n";
        assert_eq!(
            parse_http_request(status),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: true,
                },
                consumed: status.len(),
            }
        );

        let options = b"OPTIONS /control HTTP/1.1\r\n\r\n";
        assert_eq!(
            parse_http_request(options),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Options,
                    keep_alive: true,
                },
                consumed: options.len(),
            }
        );

        for req in [
            b"PUT /control HTTP/1.1\r\n\r\n".as_slice(),
            b"DELETE /status HTTP/1.1\r\n\r\n".as_slice(),
            b"HEAD / HTTP/1.1\r\n\r\n".as_slice(),
            b"POST /status HTTP/1.1\r\n\r\n".as_slice(),
            b"GET /unknown HTTP/1.1\r\n\r\n".as_slice(),
        ] {
            assert_eq!(
                parse_http_request(req),
                ParseProgress::Complete {
                    request: ParsedHttpRequest {
                        route: HttpRoute::NotFound,
                        keep_alive: true,
                    },
                    consumed: req.len(),
                }
            );
        }
    }

    #[test]
    fn test_http_versions_and_connection_semantics() {
        // HTTP/1.0 defaults to keep_alive = false.
        let http10_default = b"GET /status HTTP/1.0\r\n\r\n";
        assert_eq!(
            parse_http_request(http10_default),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: false,
                },
                consumed: http10_default.len(),
            }
        );

        // HTTP/1.0 with Connection: keep-alive enables keep_alive.
        let http10_ka = b"GET /status HTTP/1.0\r\nConnection: keep-alive\r\n\r\n";
        assert_eq!(
            parse_http_request(http10_ka),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: true,
                },
                consumed: http10_ka.len(),
            }
        );

        // Content-Length: 0 preserves keep_alive on HTTP/1.1.
        let cl_zero = b"POST /control?t=100 HTTP/1.1\r\nContent-Length: 0\r\n\r\n";
        match parse_http_request(cl_zero) {
            ParseProgress::Complete { request, .. } => assert!(request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        // Invalid Content-Length disables keep_alive (unwrap_or(1) > 0).
        let cl_invalid = b"POST /control?t=100 HTTP/1.1\r\nContent-Length: abc\r\n\r\n";
        match parse_http_request(cl_invalid) {
            ParseProgress::Complete { request, .. } => assert!(!request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        // Transfer-Encoding disables keep_alive.
        let te = b"POST /control?t=100 HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n";
        match parse_http_request(te) {
            ParseProgress::Complete { request, .. } => assert!(!request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        // Conflicting headers: later Connection header overrides earlier one; Content-Length > 0 disables keep_alive.
        let conflicting =
            b"GET /status HTTP/1.1\r\nConnection: close\r\nConnection: keep-alive\r\n\r\n";
        match parse_http_request(conflicting) {
            ParseProgress::Complete { request, .. } => assert!(request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        let ka_then_cl =
            b"GET /status HTTP/1.1\r\nConnection: keep-alive\r\nContent-Length: 5\r\n\r\n";
        match parse_http_request(ka_then_cl) {
            ParseProgress::Complete { request, .. } => assert!(!request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        // Content-Length > 0 before Connection: keep-alive must still close so unread body bytes cannot desync.
        let cl_then_ka =
            b"POST /control?t=100 HTTP/1.1\r\nContent-Length: 5\r\nConnection: keep-alive\r\n\r\n";
        match parse_http_request(cl_then_ka) {
            ParseProgress::Complete { request, .. } => assert!(!request.keep_alive),
            other => panic!("unexpected: {:?}", other),
        }

        // Unsupported HTTP versions are rejected as BadRequest.
        assert_eq!(
            parse_http_request(b"GET / HTTP/2.0\r\n\r\n"),
            ParseProgress::BadRequest
        );
        assert_eq!(
            parse_http_request(b"GET / HTTP/0.9\r\n\r\n"),
            ParseProgress::BadRequest
        );
    }

    #[test]
    fn test_http_parser_boundaries_pipelining_and_case() {
        // Empty input -> Incomplete.
        assert_eq!(parse_http_request(b""), ParseProgress::Incomplete);

        // LF-only (\n\n) header terminator is accepted.
        let lf_only = b"GET /status HTTP/1.1\nCONNECTION: CLOSE\ncOnTeNt-LeNgTh: 0\n\n";
        assert_eq!(
            parse_http_request(lf_only),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: false,
                },
                consumed: lf_only.len(),
            }
        );

        // Non-UTF-8 bytes in headers -> BadRequest.
        assert_eq!(
            parse_http_request(b"GET /\xFF\xFE HTTP/1.1\r\n\r\n"),
            ParseProgress::BadRequest
        );

        // Exact MAX_HTTP_HEADER_BYTES with a complete request inside -> Complete.
        let mut exact_max = [b'X'; MAX_HTTP_HEADER_BYTES];
        let prefix = b"GET /status HTTP/1.1\r\n\r\n";
        exact_max[..prefix.len()].copy_from_slice(prefix);
        assert_eq!(
            parse_http_request(&exact_max),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: true,
                },
                consumed: prefix.len(),
            }
        );

        // HTTP pipelining: two back-to-back requests in one buffer.
        let pipelined = b"GET / HTTP/1.1\r\n\r\nGET /status HTTP/1.1\r\nConnection: close\r\n\r\n";
        let (req1, consumed1) = match parse_http_request(pipelined) {
            ParseProgress::Complete { request, consumed } => (request, consumed),
            other => panic!("unexpected: {:?}", other),
        };
        assert_eq!(req1.route, HttpRoute::Index);
        assert!(req1.keep_alive);
        assert_eq!(consumed1, b"GET / HTTP/1.1\r\n\r\n".len());

        let (req2, consumed2) = match parse_http_request(&pipelined[consumed1..]) {
            ParseProgress::Complete { request, consumed } => (request, consumed),
            other => panic!("unexpected: {:?}", other),
        };
        assert_eq!(req2.route, HttpRoute::Status);
        assert!(!req2.keep_alive);
        assert_eq!(consumed1 + consumed2, pipelined.len());

        // Malformed request lines: extra token, missing version, whitespace-only.
        assert_eq!(
            parse_http_request(b"GET / HTTP/1.1 EXTRA\r\n\r\n"),
            ParseProgress::BadRequest
        );
        assert_eq!(
            parse_http_request(b"GET /\r\n\r\n"),
            ParseProgress::BadRequest
        );
        assert_eq!(
            parse_http_request(b"   \r\n\r\n"),
            ParseProgress::BadRequest
        );

        // Malformed header line without ':' is ignored without failing valid request.
        let weird_hdr = b"GET /status HTTP/1.1\r\nMalformedHeaderWithoutColon\r\n\r\n";
        assert_eq!(
            parse_http_request(weird_hdr),
            ParseProgress::Complete {
                request: ParsedHttpRequest {
                    route: HttpRoute::Status,
                    keep_alive: true,
                },
                consumed: weird_hdr.len(),
            }
        );
    }

    #[test]
    fn test_http_control_query_edge_cases() {
        // Empty query returns default DriveCommand.
        assert_eq!(parse_control_query(""), DriveCommand::default());

        // Missing parameters retain defaults.
        let partial = parse_control_query("t=250");
        assert!((partial.throttle - 0.25).abs() < 1e-4);
        assert_eq!(partial.steering, 0.0);
        assert_eq!(partial.gear, SpeedGear::Normal);
        assert!(!partial.brake);
        assert!(partial.adas_enabled);

        // Invalid numeric values fall back to 0 / Normal gear.
        let invalid_nums = parse_control_query("t=abc&s=1.5&g=fast");
        assert_eq!(invalid_nums.throttle, 0.0);
        assert_eq!(invalid_nums.steering, 0.0);
        assert_eq!(invalid_nums.gear, SpeedGear::Normal);

        // Out-of-range gear values fall back to Normal (SpeedGear::from_u8 contract).
        assert_eq!(parse_control_query("g=0").gear, SpeedGear::Normal);
        assert_eq!(parse_control_query("g=99").gear, SpeedGear::Normal);
        assert_eq!(parse_control_query("g=1").gear, SpeedGear::Crawl);

        // Boolean flags: b=true, a=false, and invalid boolean values.
        let bools = parse_control_query("b=true&a=false");
        assert!(bools.brake);
        assert!(!bools.adas_enabled);

        let invalid_bools = parse_control_query("b=yes&a=yes");
        assert!(!invalid_bools.brake);
        assert!(invalid_bools.adas_enabled);

        // Duplicate parameters: last value wins.
        let dupes = parse_control_query("t=200&t=-500&g=1&g=3&a=0&a=1");
        assert!((dupes.throttle + 0.5).abs() < 1e-4);
        assert_eq!(dupes.gear, SpeedGear::Sport);
        assert!(dupes.adas_enabled);

        // URL-encoded values are not decoded by the parser, so numeric parse falls back to 0.
        let url_encoded = parse_control_query("t=%2B500&s=100%20");
        assert_eq!(url_encoded.throttle, 0.0);
        assert_eq!(url_encoded.steering, 0.0);

        // Whitespace and malformed separators (empty segments, missing '=', missing key).
        let malformed = parse_control_query("&&badpair&=123&t= 500&s=-300&");
        assert_eq!(malformed.throttle, 0.0); // " 500" fails i32::from_str
        assert!((malformed.steering + 0.3).abs() < 1e-4);
    }
}
