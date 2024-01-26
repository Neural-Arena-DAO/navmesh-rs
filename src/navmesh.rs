use recastnavigation_sys::{dtNavMesh, dtFreeNavMesh};
use super::import::mset::from_mset;

#[derive(Debug)]
pub struct NavMesh {
    pub nav: *mut dtNavMesh,
}

impl Drop for NavMesh {
    fn drop(
        &mut self
    ) {
        unsafe { dtFreeNavMesh(self.nav) }
    }
}

impl NavMesh {
    pub fn from_bytes(
        buf: &[u8]
    ) -> Self {
        match from_mset(buf) {
            Err(msg) => panic!("{}", msg),
            Ok(navmesh) => {
                Self {
                    nav: navmesh as _
                }
            }
        }
    }
}

