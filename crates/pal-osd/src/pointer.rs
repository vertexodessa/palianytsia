//! Pointer input helpers for ray-driven front ends (the Quest).
/// Hand jitter on a ray moves the cursor more than egui's 6 points between
/// trigger press and release, and egui then drops the click (or, in a
/// scroll area, starts a drag). From the press until the release the cursor
/// egui sees stays where the press landed, unless the ray leaves by more
/// than `RADIUS`: then it follows (a deliberate drag, e.g. a scroll) and
/// the press is not a click any more.
#[derive(Default)]
pub struct ClickLock {
    origin: Option<egui::Pos2>,
    free: bool,
}

impl ClickLock {
    const RADIUS: f32 = 28.0;

    /// Where egui sees the pointer this frame: `p` is the ray's point,
    /// `click` the trigger edge (Some(true) press, Some(false) release).
    pub fn place(&mut self, p: egui::Pos2, click: Option<bool>) -> egui::Pos2 {
        match click {
            Some(true) => {
                self.origin = Some(p);
                self.free = false;
                p
            }
            Some(false) => {
                let at = self.held(p);
                self.origin = None;
                at
            }
            None => self.held(p),
        }
    }

    fn held(&mut self, p: egui::Pos2) -> egui::Pos2 {
        match self.origin {
            Some(o) if !self.free => {
                if o.distance(p) > Self::RADIUS {
                    self.free = true;
                    p
                } else {
                    o
                }
            }
            _ => p,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::ClickLock;
    use egui::pos2;

    #[test]
    fn a_shaky_click_lands_where_it_was_pressed() {
        let mut l = ClickLock::default();
        assert_eq!(l.place(pos2(100.0, 100.0), None), pos2(100.0, 100.0));
        assert_eq!(l.place(pos2(100.0, 100.0), Some(true)), pos2(100.0, 100.0));
        assert_eq!(l.place(pos2(112.0, 95.0), None), pos2(100.0, 100.0));
        assert_eq!(l.place(pos2(90.0, 118.0), Some(false)), pos2(100.0, 100.0));
        // Released: the cursor follows the ray again.
        assert_eq!(l.place(pos2(90.0, 118.0), None), pos2(90.0, 118.0));
    }

    #[test]
    fn a_long_move_while_held_is_a_drag() {
        let mut l = ClickLock::default();
        l.place(pos2(100.0, 100.0), Some(true));
        assert_eq!(l.place(pos2(100.0, 140.0), None), pos2(100.0, 140.0));
        // Back inside the radius: still free, the drag goes on.
        assert_eq!(l.place(pos2(100.0, 110.0), None), pos2(100.0, 110.0));
        assert_eq!(l.place(pos2(100.0, 105.0), Some(false)), pos2(100.0, 105.0));
    }
}
