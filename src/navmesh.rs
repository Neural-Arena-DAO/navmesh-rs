use recastnavigation_sys::{dtNavMesh, dtFreeNavMesh};
use super::serdes::import_from_array;

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
    pub fn from_array(
        buf: Vec<u8>
    ) -> Self {
        match import_from_array(buf) {
            Err(msg) => panic!("{}", msg),
            Ok(navmesh) => {
                Self {
                    nav: navmesh as _
                }
            }
        }
    }
}

