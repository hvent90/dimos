// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Dynamic entity primitives that piggyback on the static scene raycast.
// Descriptors arrive once in sim2.WorldManifest; poses and authoritative
// episode/tick metadata arrive in sim2.WorldStateFrame.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::accel::{Bvh, Triangle, RAY_EPSILON};
use glam::{Mat4, Quat, Vec3};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct EntityDescriptor {
    pub entity_id: String,
    #[serde(default)]
    pub kind: String,
    #[serde(default)]
    pub mesh_ref: String,
    #[serde(default = "default_shape")]
    pub shape_hint: String,
    #[serde(default)]
    pub extents: Vec<f32>,
}

#[derive(Debug, Deserialize)]
pub struct WorldManifest {
    version: u32,
    pub entities: Vec<EntityDescriptor>,
}

impl Default for WorldManifest {
    fn default() -> Self {
        Self {
            version: 1,
            entities: Vec::new(),
        }
    }
}

impl WorldManifest {
    pub fn decode(bytes: &[u8]) -> std::io::Result<Self> {
        let manifest: Self = decode_json(bytes, "WorldManifest")?;
        if manifest.version != 1 {
            return Err(invalid_data(format!(
                "unsupported WorldManifest version {}",
                manifest.version
            )));
        }
        Ok(manifest)
    }
}

#[derive(Debug, Deserialize)]
pub struct EntityPose {
    pub entity_id: String,
    pub position: [f32; 3],
    pub quaternion_xyzw: [f32; 4],
}

#[derive(Debug, Deserialize)]
pub struct WorldStateFrame {
    version: u32,
    pub episode_id: u64,
    pub physics_tick: u64,
    pub sim_time: f64,
    pub entities: Vec<EntityPose>,
}

impl WorldStateFrame {
    pub fn decode(bytes: &[u8]) -> std::io::Result<Self> {
        let frame: Self = decode_json(bytes, "WorldStateFrame")?;
        if !matches!(frame.version, 1 | 2) {
            return Err(invalid_data(format!(
                "unsupported WorldStateFrame version {}",
                frame.version
            )));
        }
        Ok(frame)
    }
}

fn decode_json<T: for<'de> Deserialize<'de>>(bytes: &[u8], name: &str) -> std::io::Result<T> {
    serde_json::from_slice(bytes)
        .map_err(|error| invalid_data(format!("failed to decode {name}: {error}")))
}

fn invalid_data(message: String) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, message)
}

fn default_shape() -> String {
    "mesh".to_string()
}

#[derive(Debug, Clone)]
pub enum EntityShape {
    Box {
        half_extents: Vec3,
    },
    Sphere {
        radius: f32,
    },
    // Cylinder is the local Z axis (matches Babylon's CreateCylinder default
    // after our Z-up rotation in app.js).
    Cylinder {
        radius: f32,
        half_height: f32,
    },
    Mesh {
        mesh_ref: String,
        mesh: Option<Arc<MeshAccel>>,
    },
}

pub struct Entity {
    // Kept around for future logging / debugging; not read by the raycast hot path.
    #[allow(dead_code)]
    pub id: String,
    pub shape: EntityShape,
    // world ← local. Convenience for downstream; currently unused in the
    // raycast itself since rigid transforms preserve distance.
    #[allow(dead_code)]
    pub world_from_local: Mat4,
    // local ← world. Cached so each ray cheaply moves into the entity's
    // local frame before the analytical test.
    pub local_from_world: Mat4,
}

pub fn entities_from_world(manifest: &WorldManifest, frame: &WorldStateFrame) -> Vec<Entity> {
    let descriptors: HashMap<&str, &EntityDescriptor> = manifest
        .entities
        .iter()
        .map(|descriptor| (descriptor.entity_id.as_str(), descriptor))
        .collect();
    frame
        .entities
        .iter()
        .filter_map(|pose| {
            let descriptor = descriptors.get(pose.entity_id.as_str())?;
            if descriptor.kind == "static" {
                return None;
            }
            if descriptor.shape_hint == "mesh" && descriptor.mesh_ref.is_empty() {
                return None;
            }
            let shape = parse_shape(
                &descriptor.shape_hint,
                &descriptor.extents,
                &descriptor.mesh_ref,
            )?;
            let translation = Vec3::from_array(pose.position);
            let rotation = Quat::from_array(pose.quaternion_xyzw);
            let world_from_local = Mat4::from_rotation_translation(rotation, translation);
            Some(Entity {
                id: pose.entity_id.clone(),
                shape,
                world_from_local,
                local_from_world: world_from_local.inverse(),
            })
        })
        .collect()
}

fn parse_shape(name: &str, extents: &[f32], mesh_ref: &str) -> Option<EntityShape> {
    match name {
        "box" => {
            // (w, h, d) full extents → half extents.
            let w = extents.first().copied().unwrap_or(1.0);
            let h = extents.get(1).copied().unwrap_or(1.0);
            let d = extents.get(2).copied().unwrap_or(1.0);
            Some(EntityShape::Box {
                half_extents: Vec3::new(w * 0.5, h * 0.5, d * 0.5),
            })
        }
        "sphere" => {
            let r = extents.first().copied().unwrap_or(0.5);
            Some(EntityShape::Sphere { radius: r })
        }
        "cylinder" => {
            let r = extents.first().copied().unwrap_or(0.5);
            let h = extents.get(1).copied().unwrap_or(1.0);
            Some(EntityShape::Cylinder {
                radius: r,
                half_height: h * 0.5,
            })
        }
        "mesh" => {
            if mesh_ref.is_empty() {
                eprintln!("scene_lidar: mesh entity missing mesh_ref");
                None
            } else {
                Some(EntityShape::Mesh {
                    mesh_ref: mesh_ref.to_string(),
                    mesh: None,
                })
            }
        }
        other => {
            eprintln!("scene_lidar: unknown entity shape {other:?}");
            None
        }
    }
}

#[derive(Debug, Default)]
pub struct MeshAccel {
    triangles: Vec<Triangle>,
    bvh: Bvh,
}

impl MeshAccel {
    fn load(path: &Path) -> std::io::Result<Self> {
        let (document, buffers, _) = gltf::import(path).map_err(|error| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("failed to import {}: {error}", path.display()),
            )
        })?;
        let mut triangles = Vec::new();
        for scene in document.scenes() {
            for node in scene.nodes() {
                collect_node_triangles(node, Mat4::IDENTITY, &buffers, &mut triangles);
            }
        }
        if triangles.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("entity mesh has no triangles: {}", path.display()),
            ));
        }
        let bvh = Bvh::build(&triangles);
        Ok(Self { triangles, bvh })
    }

    fn raycast(&self, origin: Vec3, direction: Vec3, max_range: f32) -> Option<(Vec3, f32)> {
        self.bvh
            .raycast(origin, direction, max_range, &self.triangles)
    }

    fn triangle_count(&self) -> usize {
        self.triangles.len()
    }
}

#[derive(Debug, Default)]
pub struct MeshCache {
    asset_root: PathBuf,
    meshes: HashMap<String, Option<Arc<MeshAccel>>>,
}

impl MeshCache {
    pub fn new(asset_root: impl Into<PathBuf>) -> Self {
        Self {
            asset_root: asset_root.into(),
            meshes: HashMap::new(),
        }
    }

    pub fn resolve_entities(&mut self, entities: &mut [Entity]) {
        for entity in entities {
            let EntityShape::Mesh { mesh_ref, mesh } = &mut entity.shape else {
                continue;
            };
            *mesh = self.mesh_for(mesh_ref);
        }
    }

    fn mesh_for(&mut self, mesh_ref: &str) -> Option<Arc<MeshAccel>> {
        if let Some(cached) = self.meshes.get(mesh_ref) {
            return cached.clone();
        }
        let path = resolve_asset_path(&self.asset_root, mesh_ref);
        let loaded = match MeshAccel::load(&path) {
            Ok(mesh) => {
                eprintln!(
                    "scene_lidar: loaded entity mesh {} ({} triangles) from {}",
                    mesh_ref,
                    mesh.triangle_count(),
                    path.display()
                );
                Some(Arc::new(mesh))
            }
            Err(error) => {
                eprintln!(
                    "scene_lidar: failed to load entity mesh {} from {}: {}",
                    mesh_ref,
                    path.display(),
                    error
                );
                None
            }
        };
        self.meshes.insert(mesh_ref.to_string(), loaded.clone());
        loaded
    }
}

fn resolve_asset_path(asset_root: &Path, mesh_ref: &str) -> PathBuf {
    let path = PathBuf::from(mesh_ref);
    if path.is_absolute() {
        path
    } else {
        asset_root.join(path)
    }
}

fn collect_node_triangles(
    node: gltf::Node<'_>,
    parent_transform: Mat4,
    buffers: &[gltf::buffer::Data],
    out: &mut Vec<Triangle>,
) {
    let local_transform = node_transform(&node);
    let node_transform = parent_transform * local_transform;
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()].0));
            let Some(positions_iter) = reader.read_positions() else {
                continue;
            };
            let positions: Vec<Vec3> = positions_iter.map(Vec3::from_array).collect();
            if positions.len() < 3 {
                continue;
            }
            let indices: Vec<usize> = match reader.read_indices() {
                Some(iter) => iter.into_u32().map(|i| i as usize).collect(),
                None => (0..positions.len()).collect(),
            };
            for tri in indices.chunks_exact(3) {
                let a = node_transform.transform_point3(positions[tri[0]]);
                let b = node_transform.transform_point3(positions[tri[1]]);
                let c = node_transform.transform_point3(positions[tri[2]]);
                if (b - a).cross(c - a).length_squared() > RAY_EPSILON {
                    out.push(Triangle::new(a, b, c));
                }
            }
        }
    }
    for child in node.children() {
        collect_node_triangles(child, node_transform, buffers, out);
    }
}

fn node_transform(node: &gltf::Node<'_>) -> Mat4 {
    let (translation, rotation, scale) = node.transform().decomposed();
    Mat4::from_scale_rotation_translation(
        Vec3::from_array(scale),
        Quat::from_xyzw(rotation[0], rotation[1], rotation[2], rotation[3]),
        Vec3::from_array(translation),
    )
}

/// Cast `direction` from `origin` against a single entity. Direction must
/// be unit length in world space; rotation in the entity transform is
/// orthonormal, so the rotated local direction is also unit and the
/// returned local distance equals the world distance.
///
/// Returns (world hit point, distance) on hit, None on miss / past max_range.
pub fn raycast(
    entity: &Entity,
    origin: Vec3,
    direction: Vec3,
    max_range: f32,
) -> Option<(Vec3, f32)> {
    let local_origin = entity.local_from_world.transform_point3(origin);
    let local_dir = entity.local_from_world.transform_vector3(direction);
    let dist = match &entity.shape {
        EntityShape::Box { half_extents } => {
            ray_box(local_origin, local_dir, *half_extents, max_range)
        }
        EntityShape::Sphere { radius } => ray_sphere(local_origin, local_dir, *radius, max_range),
        EntityShape::Cylinder {
            radius,
            half_height,
        } => ray_cylinder_z(local_origin, local_dir, *radius, *half_height, max_range),
        EntityShape::Mesh {
            mesh: Some(mesh), ..
        } => {
            let (local_hit, _) = mesh.raycast(local_origin, local_dir, max_range)?;
            let hit_world = entity.world_from_local.transform_point3(local_hit);
            let dist = hit_world.distance(origin);
            if dist <= max_range {
                return Some((hit_world, dist));
            }
            return None;
        }
        EntityShape::Mesh { mesh: None, .. } => return None,
    }?;
    let hit_world = origin + direction * dist;
    Some((hit_world, dist))
}

/// Slab method against an AABB centered at origin with `half_extents`.
fn ray_box(origin: Vec3, direction: Vec3, half_extents: Vec3, max_range: f32) -> Option<f32> {
    let mut tmin: f32 = 0.0;
    let mut tmax: f32 = max_range;
    for axis in 0..3 {
        let o = origin[axis];
        let d = direction[axis];
        let h = half_extents[axis];
        if d.abs() < 1.0e-8 {
            // Ray parallel to slab: must already be between the planes.
            if o < -h || o > h {
                return None;
            }
        } else {
            let inv = 1.0 / d;
            let mut t1 = (-h - o) * inv;
            let mut t2 = (h - o) * inv;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            if t1 > tmin {
                tmin = t1;
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmin > 0.0 {
        Some(tmin)
    } else if tmax > 0.0 {
        // Ray origin is inside the box — surface hit on the way out.
        Some(tmax)
    } else {
        None
    }
}

fn ray_sphere(origin: Vec3, direction: Vec3, radius: f32, max_range: f32) -> Option<f32> {
    let oc = origin;
    let b = oc.dot(direction);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let t = -b - s;
    if t > 0.0 && t <= max_range {
        return Some(t);
    }
    let t = -b + s;
    if t > 0.0 && t <= max_range {
        return Some(t);
    }
    None
}

/// Capped cylinder along local Z axis: |z| <= half_height, x² + y² <= r².
fn ray_cylinder_z(
    origin: Vec3,
    direction: Vec3,
    radius: f32,
    half_height: f32,
    max_range: f32,
) -> Option<f32> {
    let dxy_sq = direction.x * direction.x + direction.y * direction.y;
    let mut best: Option<f32> = None;
    let mut consider = |t: f32| {
        if t > 0.0 && t <= max_range {
            best = Some(match best {
                Some(b) if b < t => b,
                _ => t,
            });
        }
    };

    // Side surface
    if dxy_sq > 1.0e-12 {
        let a = dxy_sq;
        let b = origin.x * direction.x + origin.y * direction.y;
        let c = origin.x * origin.x + origin.y * origin.y - radius * radius;
        let disc = b * b - a * c;
        if disc >= 0.0 {
            let s = disc.sqrt();
            for t in [(-b - s) / a, (-b + s) / a] {
                let z = origin.z + direction.z * t;
                if z >= -half_height && z <= half_height {
                    consider(t);
                }
            }
        }
    }

    // End caps (z = ±half_height)
    if direction.z.abs() > 1.0e-8 {
        for cap in [-half_height, half_height] {
            let t = (cap - origin.z) / direction.z;
            let x = origin.x + direction.x * t;
            let y = origin.y + direction.y * t;
            if x * x + y * y <= radius * radius {
                consider(t);
            }
        }
    }

    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joins_typed_manifest_and_world_frame() {
        let manifest = WorldManifest::decode(
            br#"{"version":1,"scene_revision":"test","frame_id":"world","entities":[{"entity_id":"box_1","kind":"dynamic","backend_name":"box_1","mesh_ref":"","shape_hint":"box","extents":[0.8,0.8,1.2],"mass":8.0}]}"#,
        )
        .unwrap();
        let frame = WorldStateFrame::decode(
            br#"{"version":2,"episode_id":3,"physics_tick":7,"control_tick":2,"sim_time":0.035,"scene_revision":"test","entities":[{"entity_id":"box_1","frame_id":"world","position":[2.0,0.0,0.6],"quaternion_xyzw":[0.0,0.0,0.0,1.0],"linear_velocity":[0.0,0.0,0.0],"angular_velocity":[0.0,0.0,0.0]}]}"#,
        )
        .unwrap();

        let entities = entities_from_world(&manifest, &frame);
        assert_eq!(entities.len(), 1);
        assert_eq!(frame.episode_id, 3);
        assert_eq!(frame.physics_tick, 7);
        let hit = raycast(&entities[0], Vec3::new(0.0, 0.0, 0.6), Vec3::X, 10.0);
        let (_, dist) = hit.expect("ray should hit the decoded box");
        assert!((dist - 1.6).abs() < 1.0e-4);
    }

    #[test]
    fn rejects_unknown_manifest_version() {
        let error = WorldManifest::decode(br#"{"version":2,"entities":[]}"#).unwrap_err();
        assert!(error
            .to_string()
            .contains("unsupported WorldManifest version"));
    }

    #[test]
    fn joins_mesh_descriptor_with_pose() {
        let manifest = WorldManifest::decode(
            br#"{"version":1,"scene_revision":"test","frame_id":"world","entities":[{"entity_id":"chair_1","kind":"dynamic","backend_name":"chair_1","mesh_ref":"entities/chair_1/visual.glb","shape_hint":"mesh","extents":[],"mass":8.0}]}"#,
        )
        .unwrap();
        let frame = WorldStateFrame::decode(
            br#"{"version":2,"episode_id":1,"physics_tick":1,"control_tick":1,"sim_time":0.02,"scene_revision":"test","entities":[{"entity_id":"chair_1","position":[2.0,0.0,0.6],"quaternion_xyzw":[0.0,0.0,0.0,1.0]}]}"#,
        )
        .unwrap();

        let entities = entities_from_world(&manifest, &frame);
        assert_eq!(entities.len(), 1);
        match &entities[0].shape {
            EntityShape::Mesh { mesh_ref, mesh } => {
                assert_eq!(mesh_ref, "entities/chair_1/visual.glb");
                assert!(mesh.is_none());
            }
            _ => panic!("expected mesh entity"),
        }
    }
}
