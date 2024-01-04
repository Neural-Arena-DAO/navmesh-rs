use std::{os::raw::c_int, mem::size_of, ptr::{null_mut, copy_nonoverlapping, addr_of_mut}};
use recastnavigation_sys::{dtNavMeshParams, dtTileRef, dtNavMesh, dtAlloc, dtAllocHint_DT_ALLOC_PERM, dtTileFlags_DT_TILE_FREE_DATA, dtAllocNavMesh, DT_SUCCESS};

const NAVMESHSET_MAGIC: c_int = ('M' as c_int) << 24 | ('S' as c_int) << 16 | ('E' as c_int) << 8 | ('T' as c_int); //'MSET'
const NAVMESHSET_VERSION: c_int = 1;

#[repr(C)]
struct RecastHeader {
    magic: c_int,
    version: c_int,
    num_tiles: c_int,
}

#[repr(C)]
struct NavMeshTileHeader {
    tile_ref: dtTileRef,
    data_size: c_int
}

pub(crate) fn import_from_array(
    buf: Vec<u8>
) -> Result<*const dtNavMesh, String> {

    unsafe { 
        let mut i = 0;

        // load header
        let mut recast_header = RecastHeader {
            magic: 0,
            version: 0,
            num_tiles: 0,
        };
        copy_nonoverlapping(&buf[i] as *const u8, addr_of_mut!(recast_header) as _, size_of::<RecastHeader>());
        i += size_of::<RecastHeader>();

        // do basic checks
        if recast_header.magic != NAVMESHSET_MAGIC {
            return Err("Buffer is not a Navmesh Set binary".to_string());
        }

        if recast_header.version != NAVMESHSET_VERSION {
            return Err("Unsupported Navmesh version".to_string());
        }

        // load mesh params
        let mut params = dtNavMeshParams {
            orig: [0.0f32; 3],
            tileWidth: 0.0,
            tileHeight: 0.0,
            maxTiles: 0,
            maxPolys: 0,
        };
        copy_nonoverlapping(&buf[i] as *const u8, addr_of_mut!(params) as _, size_of::<dtNavMeshParams>());
        i += size_of::<dtNavMeshParams>();
        
        // create a nav mesh
        let nav_mesh = dtAllocNavMesh();
        if nav_mesh.as_mut().unwrap().init(&params as *const dtNavMeshParams) != DT_SUCCESS {
            return Err("Could not initialize Navmesh".to_string());
        }

        for _ in 0..recast_header.num_tiles {
            // load tile header
            let mut tile_header = NavMeshTileHeader {
                data_size: 0,
                tile_ref: 0
            };
            copy_nonoverlapping(&buf[i] as *const u8, addr_of_mut!(tile_header) as _, size_of::<NavMeshTileHeader>());
            i += size_of::<NavMeshTileHeader>();

            // EOL?
            if tile_header.tile_ref == 0 || tile_header.data_size == 0 {
                break;
            }

            let data = dtAlloc(tile_header.data_size as usize, dtAllocHint_DT_ALLOC_PERM) as *mut u8;
            if data.is_null() {
                return Err("Could not allocated memory for Navmesh tile".to_string());
            }

            data.copy_from(buf.as_ptr().add(i) as _, tile_header.data_size as usize);
            i += tile_header.data_size as usize;

            if nav_mesh.as_mut().unwrap().addTile(data, tile_header.data_size, dtTileFlags_DT_TILE_FREE_DATA, tile_header.tile_ref, null_mut()) != DT_SUCCESS {
                return Err("Could not add tile to Navmesh".to_string());
            }
        }

        Ok(nav_mesh)
    }
}