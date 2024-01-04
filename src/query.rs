use std::ptr::{null_mut, addr_of_mut};
use recastnavigation_sys::{dtNavMeshQuery, dtPolyRef, dtQueryFilter, DT_SUCCESS};
use three_rs::math::vector3::Vector3;

const EXTENTS: [f32; 3] = [1.0, 1.0, 1.0];
const FILTER: dtQueryFilter = dtQueryFilter {
    m_areaCost: [1.0; 64],
    m_includeFlags: 0xffff,
    m_excludeFlags: 0,
};

fn find_poly_ref(
    query: &dtNavMeshQuery, 
    pos: &[f32; 3]
) -> dtPolyRef {

    let mut poly_ref: dtPolyRef = 0;

    unsafe {
        query.findNearestPoly(
            pos.as_ptr(),
            EXTENTS.as_ptr(),
            &FILTER,
            &mut poly_ref,
            std::ptr::null_mut(),
        );
    }

    poly_ref
}

pub fn find_closest_point_on_poly(
    query: &dtNavMeshQuery,
    poly_ref: dtPolyRef,
    point: &[f32; 3]
) -> Vector3 {
    unsafe {
        let mut pos_overlay = false;
        let mut closest = [0.0f32; 3];
        if query.closestPointOnPoly(poly_ref, point.as_ptr(), addr_of_mut!(closest) as _, addr_of_mut!(pos_overlay)) != DT_SUCCESS {
            Vector3::zero()
        }
        else {
            Vector3::from_slice(&closest)
        }
    }
}

pub fn find_closest_point(
    query: &dtNavMeshQuery,
    pos: &Vector3
) -> Vector3 {
    unsafe {
        let point = pos.to_slice();
        let mut poly_ref: dtPolyRef = 0;
        if query.findNearestPoly(point.as_ptr(), EXTENTS.as_ptr(), &FILTER, addr_of_mut!(poly_ref), null_mut()) != DT_SUCCESS {
            Vector3::zero()
        }
        else {
            find_closest_point_on_poly(query, poly_ref, &point)
        }
    }
}

pub fn find_path(
    query: &dtNavMeshQuery,
    a: &Vector3,
    b: &Vector3,
    max_path_len: usize
) -> Vec<Vector3> {
    unsafe { 
        let a_point = a.to_slice();
        let b_point = b.to_slice();

        let a_ref = find_poly_ref(query, &a_point);
        let b_ref = find_poly_ref(query, &b_point);

        let mut path = vec![0; max_path_len];
        let mut path_count = 0;

        if query.findPath(
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
            find_closest_point_on_poly(query, last_poly, &b_point)
        }
        else {
            Vector3::new(b.x, b.y, b.z)
        };

        let mut straight_path = vec![0.0f32; max_path_len * 3];
        let mut straight_path_flags = vec![0u8; max_path_len];
        let mut straight_path_refs = vec![0u32; max_path_len];
        let mut straight_path_count = 0;

        if query.findStraightPath(
            a_point.as_ptr(),
            closest_end.to_slice().as_ptr(),
            path.as_ptr(),
            path_count,
            straight_path.as_mut_ptr(),
            straight_path_flags.as_mut_ptr(),
            straight_path_refs.as_mut_ptr(),
            &mut straight_path_count,
            max_path_len as _,
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