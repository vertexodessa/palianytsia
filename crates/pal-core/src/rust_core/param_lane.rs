//! The parameter client's wiring (WebGs's ParamClient): it follows the
//! session (re-lists on every edge and on a new vtx_nonce while linked),
//! and its requests leave through the MAVLink tunnel. Also the table's JSON
//! for the UI.
use crate::json::J;
use crate::mav::{MavTunnel, ParamClient};

#[derive(Default)]
pub(super) struct ParamLane {
    pub client: ParamClient,
    linked: bool,
    vtx_seen: u32,
}

impl ParamLane {
    /// Per tick, before the tunnel flushes, so a re-list leaves this tick.
    pub fn tick(&mut self, linked: bool, vtx_nonce: u32, now_ms: u64, mav: &mut MavTunnel) {
        if linked != self.linked || (linked && vtx_nonce != self.vtx_seen) {
            if self.linked {
                self.client.set_linked(false, now_ms);
            }
            if linked {
                self.client.set_linked(true, now_ms);
            }
            self.linked = linked;
            self.vtx_seen = vtx_nonce;
        }
        self.client.tick(now_ms);
        self.drain(mav);
    }

    /// The client's requests onto the tunnel (WebGs pushes as it sends).
    pub fn drain(&mut self, mav: &mut MavTunnel) {
        for p in self.client.take_out() {
            mav.push_outbound(&p);
        }
    }
}

/// `webgs::params_json`: the table with the client's state, or (Spotter,
/// no client) the static table with every value unknown.
pub(super) fn params_json(pc: Option<&ParamClient>, now_ms: u64) -> String {
    use crate::mav::{Apply, ListState, PType, RowState, TABLE};
    fn ty(t: PType) -> &'static str {
        match t {
            PType::Bool => "bool",
            PType::Int32 => "int",
            PType::Real32 => "real",
            PType::Str => "str",
        }
    }
    fn apply(a: Apply) -> &'static str {
        match a {
            Apply::Knob => "knob",
            Apply::Reprogram => "reprogram",
            Apply::Rebuild => "rebuild",
        }
    }
    let mut j = J::obj();
    let mut rows = J::arr();
    let st = pc.map(ParamClient::state);
    for (i, row) in TABLE.iter().enumerate() {
        let mut r = J::obj();
        r.set("id", row.id);
        r.set("section", row.section);
        r.set("key", row.key);
        r.set("type", ty(row.ty));
        r.set("apply", apply(row.apply));
        r.set("help", row.help);
        match st {
            None => {
                r.set("value", J::Null);
                r.set("state", "idle");
                r.set("status", "");
                r.set("age_ms", J::Null);
            }
            Some(st) => {
                let rs = &st.rows[i];
                r.set("value", rs.value.as_ref().map(crate::mav::format_text));
                r.set("state", rs.st.as_str());
                r.set("status", rs.status.as_str());
                let terminal = matches!(
                    rs.st,
                    RowState::Accepted
                        | RowState::Failed
                        | RowState::Unsupported
                        | RowState::Timeout
                );
                r.set(
                    "age_ms",
                    terminal.then(|| now_ms.wrapping_sub(rs.result_ms) as i64),
                );
            }
        }
        rows.push(r);
    }
    match (pc, st) {
        (Some(pc), Some(st)) => {
            j.set("linked", st.linked);
            j.set("alive", (st.alive_ms != 0).then(|| pc.alive(now_ms)));
            j.set(
                "list",
                match st.list {
                    ListState::None => "none",
                    ListState::Pending => "pending",
                    ListState::Complete => "complete",
                    ListState::Partial => "partial",
                },
            );
            j.set("mismatch", st.table_mismatch);
            j.set("pending", st.pending_row.map_or(-1, |r| r as i64));
        }
        _ => {
            j.set("linked", false);
            j.set("mode", "spotter");
        }
    }
    j.set("rows", rows);
    j.dump()
}
