use std::ptr::{null_mut, addr_of_mut};
use recastnavigation_sys::{dtNavMeshQuery, dtPolyRef, dtQueryFilter, DT_SUCCESS, dtRaycastHit};
use three_rs::math::vector3::Vector3;

use crate::navmesh::NavMesh;

const EXTENTS: [f32; 3] = [0.6, 2.0, 0.6];
const FILTER: dtQueryFilter = dtQueryFilter {
    m_areaCost: [1.0; 64],
    m_includeFlags: 0xffff,
    m_excludeFlags: 0,
};

pub struct NavMeshQuery {
    pub query: dtNavMeshQuery,
    pub max_path_len: usize,
}

impl NavMeshQuery {
    pub fn new(
        navmesh: &NavMesh,
        max_path_len: usize
    ) -> Self {
        unsafe {
            let mut query = dtNavMeshQuery::new();
            query.init(navmesh.nav, 512);
            Self {
                query,
                max_path_len
            }
        }
    }
    
    fn find_poly_ref(
        &self, 
        pos: &[f32; 3]
    ) -> dtPolyRef {

        let mut poly_ref: dtPolyRef = 0;

        unsafe {
            self.query.findNearestPoly(
                pos.as_ptr(),
                EXTENTS.as_ptr(),
                &FILTER,
                &mut poly_ref,
                std::ptr::null_mut(),
            );
        };

        poly_ref
    }

    pub fn find_closest_point_on_poly(
        &self,
        poly_ref: dtPolyRef,
        point: &[f32; 3]
    ) -> Vector3 {
        unsafe {
            let mut pos_overlay = false;
            let mut closest = [0.0f32; 3];
            if self.query.closestPointOnPoly(
                poly_ref, 
                point.as_ptr(), 
                addr_of_mut!(closest) as _, 
                addr_of_mut!(pos_overlay)
            ) != DT_SUCCESS {
                Vector3::zero()
            }
            else {
                Vector3::from_slice(&closest)
            }
        }
    }

    pub fn find_closest_point(
        &self,
        pos: &Vector3
    ) -> Vector3 {
        unsafe {
            let point = pos.to_slice();
            let mut poly_ref: dtPolyRef = 0;
            if self.query.findNearestPoly(
                point.as_ptr(), 
                EXTENTS.as_ptr(), 
                &FILTER, 
                addr_of_mut!(poly_ref), 
                null_mut()
            ) != DT_SUCCESS {
                Vector3::zero()
            }
            else {
                self.find_closest_point_on_poly(poly_ref, &point)
            }
        }
    }

    pub fn find_path(
        &self,
        a: &Vector3,
        b: &Vector3
    ) -> Vec<Vector3> {
        unsafe { 
            let a_point = a.to_slice();
            let b_point = b.to_slice();

            let a_ref = self.find_poly_ref(&a_point);
            let b_ref = self.find_poly_ref(&b_point);

            let mut path = vec![0 as dtPolyRef; self.max_path_len];
            let mut path_count = 0;

            if self.query.findPath(
                a_ref,
                b_ref,
                a_point.as_ptr(),
                b_point.as_ptr(),
                &FILTER,
                path.as_mut_ptr(),
                &mut path_count,
                path.len() as i32,
            ) != DT_SUCCESS {
                return vec![];
            }

            if path_count == 0 {
                return vec![];
            }

            let last_poly = path[path_count as usize - 1];
            let closest_end = if last_poly != b_ref {
                self.find_closest_point_on_poly(last_poly, &b_point)
            }
            else {
                Vector3::new(b.x, b.y, b.z)
            };

            let mut straight_path = vec![0.0f32; self.max_path_len * 3];
            let mut straight_path_flags = vec![0u8; self.max_path_len];
            let mut straight_path_refs = vec![0 as dtPolyRef; self.max_path_len];
            let mut straight_path_count = 0;

            if self.query.findStraightPath(
                a_point.as_ptr(),
                closest_end.to_slice().as_ptr(),
                path.as_ptr(),
                path_count,
                straight_path.as_mut_ptr(),
                straight_path_flags.as_mut_ptr(),
                straight_path_refs.as_mut_ptr(),
                &mut straight_path_count,
                self.max_path_len as _,
                0
            ) != DT_SUCCESS {
                return vec![];
            }

            let mut points = vec![];
            for i in 0..straight_path_count {
                points.push(Vector3::from_array(&straight_path, (i * 3) as _));
            }

            points
        }
    }

    pub fn raycast(
        &self,
        from: &Vector3,
        to: &Vector3
    ) -> Option<(f32, Vector3)> {
        unsafe { 
            let a_point = from.to_slice();
            let b_point = to.to_slice();

            let a_ref = self.find_poly_ref(&a_point);

            let mut hit = dtRaycastHit {
                t: 0.0,
                hitNormal: [0.0f32; 3],
                hitEdgeIndex: 0,
                path: null_mut(),
                pathCount: 0,
                maxPath: 0,
                pathCost: 0.0,
            };

            self.query.raycast1(
                a_ref,
                a_point.as_ptr(),
                b_point.as_ptr(),
                &FILTER,
                0,
                &mut hit,
                0
            );
            
            if hit.t == f32::MAX {
                None
            }
            else {
                Some((
                    hit.t, 
                    Vector3::from_slice(&hit.hitNormal)
                ))
            }
        }
    }

    pub fn distance_to_wall(
        &self,
        center: &Vector3,
        radius: f32
    ) -> Option<(f32, Vector3, Vector3)> {
        unsafe { 
            let center_point = center.to_slice();
            let center_ref = self.find_poly_ref(&center_point);
            let mut hit_dist = 0.0f32;
            let mut hit_pos = [0.0f32; 3];
            let mut hit_normal = [0.0f32; 3];

            if self.query.findDistanceToWall(
                center_ref,
                center_point.as_ptr(),
                radius,
                &FILTER,
                &mut hit_dist,
                hit_pos.as_mut_ptr(),
                hit_normal.as_mut_ptr()
            ) == DT_SUCCESS {
                if hit_dist == f32::MAX {
                    None
                }
                else {
                    if hit_dist == radius {
                        None
                    }
                    else {
                        Some((
                            hit_dist, 
                            Vector3::from_slice(&hit_pos),
                            Vector3::from_slice(&hit_normal)
                        ))
                    }
                }
            }
            else {
                None
            }
        }
    }

    pub fn get_poly_height(
        &self,
        poly_ref: dtPolyRef,
        pos: &[f32; 3]
    ) -> Option<f32> {
        unsafe {
            let mut height = 0.0f32;

            if self.query.getPolyHeight(
                poly_ref,
                pos.as_ptr(),
                &mut height
            ) == DT_SUCCESS {
                Some(height)
            }
            else {
                None
            }
        }
    }

    pub fn move_along_surface(
        &self,
        from: &Vector3,
        to: &Vector3
    ) -> Option<Vector3> {
        unsafe { 
            let a_point = from.to_slice();
            let b_point = to.to_slice();

            let a_ref = self.find_poly_ref(&a_point);

            let mut pos = [0.0f32; 3];
            let mut visited = vec![0 as dtPolyRef; self.max_path_len];
            let mut visited_count = 0;

            if self.query.moveAlongSurface(
                a_ref,
                a_point.as_ptr(),
                b_point.as_ptr(),
                &FILTER,
                pos.as_mut_ptr(),
                visited.as_mut_ptr(),
                &mut visited_count,
                self.max_path_len as _
            ) == DT_SUCCESS {
                let last_poly_ref = visited[(visited_count - 1) as usize];
                if let Some(height) = self.get_poly_height(last_poly_ref, &pos) {
                    pos[1] = height;
                }

                Some(Vector3::from_slice(&pos))
            }
            else {
                None
            }
        }
    }
}