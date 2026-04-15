/// Doppler velocity step: 17.28 cm/s per unit of `dop_index`.
const DOP_STEP_CMS: f32 = 17.28;

/// A single tracked target reported by the MR60BHA2 (from frame type 0x0A04).
///
/// Position is in the sensor's native units (metres). The coordinate system
/// has the sensor at the origin, +Y pointing forward, +X pointing right.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Target {
    /// X coordinate in metres. Positive = right of sensor, negative = left.
    pub x: f32,
    /// Y coordinate in metres. Always positive (in front of sensor).
    pub y: f32,
    /// Raw Doppler index. Use [`speed_ms`] to convert to m/s.
    pub dop_index: i32,
    /// Tracking cluster ID. Stable across frames for the same physical target.
    pub cluster_id: i32,
}

impl Target {
    /// Radial speed in m/s. Positive = approaching, negative = receding.
    pub fn speed_ms(&self) -> f32 {
        self.dop_index as f32 * DOP_STEP_CMS / 100.0
    }

    /// Euclidean distance from sensor in metres.
    pub fn dist_m(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y)
    }

    /// Angle in degrees from sensor boresight (-90 to +90).
    pub fn angle_deg(&self) -> f32 {
        libm::atan2f(self.x, self.y) * (180.0 / core::f32::consts::PI)
    }

    /// Returns true if this target slot contains valid data.
    ///
    /// An all-zero target (x=0, y=0) with a finite position is considered
    /// invalid — it means the sensor reported an empty slot.
    pub fn is_valid(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && (self.x != 0.0 || self.y != 0.0)
    }

    /// Parse a target from 16 bytes of little-endian frame data.
    ///
    /// Byte layout:
    /// ```text
    /// [0..4]   x         (f32, little-endian, metres)
    /// [4..8]   y         (f32, little-endian, metres)
    /// [8..12]  dop_index (i32, little-endian)
    /// [12..16] cluster_id (i32, little-endian)
    /// ```
    pub fn from_bytes(data: &[u8; 16]) -> Self {
        Self {
            x:          f32::from_le_bytes([data[0],  data[1],  data[2],  data[3]]),
            y:          f32::from_le_bytes([data[4],  data[5],  data[6],  data[7]]),
            dop_index:  i32::from_le_bytes([data[8],  data[9],  data[10], data[11]]),
            cluster_id: i32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        }
    }
}

/// A point cloud frame containing up to 3 simultaneously tracked targets.
///
/// Corresponds to frame types `0x0A04` (primary targets) and `0x0A08`
/// (detection-phase cloud, often empty).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointCloudFrame {
    /// Raw target slots. Slots beyond `count` are zero-initialised.
    pub targets: [Target; 3],
    /// Number of valid target slots reported in this frame (0–3).
    pub count: u8,
}

impl PointCloudFrame {
    /// Iterator over the valid (non-empty) targets in this frame.
    pub fn active_targets(&self) -> impl Iterator<Item = &Target> {
        self.targets[..self.count as usize].iter().filter(|t| t.is_valid())
    }

    /// Parse from the data section of a point cloud frame.
    ///
    /// Layout: `count: u32 (LE)` followed by `count × 16` target bytes.
    pub fn from_bytes(data: &[u8]) -> Self {
        if data.len() < 4 {
            return Self::default();
        }
        let n = u32::from_le_bytes([data[0], data[1], data[2], data[3]]) as usize;
        let count = n.min(3) as u8;
        let mut targets = [Target::default(); 3];
        for i in 0..count as usize {
            let start = 4 + i * 16;
            if start + 16 > data.len() {
                break;
            }
            let slice: &[u8; 16] = data[start..start + 16].try_into().unwrap();
            targets[i] = Target::from_bytes(slice);
        }
        PointCloudFrame { targets, count }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target_bytes(x: f32, y: f32, dop: i32, cid: i32) -> [u8; 16] {
        let mut b = [0u8; 16];
        b[0..4].copy_from_slice(&x.to_le_bytes());
        b[4..8].copy_from_slice(&y.to_le_bytes());
        b[8..12].copy_from_slice(&dop.to_le_bytes());
        b[12..16].copy_from_slice(&cid.to_le_bytes());
        b
    }

    #[test]
    fn target_roundtrip() {
        let bytes = target_bytes(0.45, 1.10, -3, 1);
        let t = Target::from_bytes(&bytes);
        assert!((t.x - 0.45).abs() < 1e-6);
        assert!((t.y - 1.10).abs() < 1e-6);
        assert_eq!(t.dop_index, -3);
        assert_eq!(t.cluster_id, 1);
    }

    #[test]
    fn target_speed_ms() {
        let bytes = target_bytes(0.0, 1.0, 10, 0);
        let t = Target::from_bytes(&bytes);
        // 10 * 17.28 / 100 = 1.728 m/s
        assert!((t.speed_ms() - 1.728).abs() < 1e-4);
    }

    #[test]
    fn target_dist_angle() {
        // target at (1.0, 1.0) → dist = sqrt(2) ≈ 1.4142, angle = atan2(1,1) = 45°
        let bytes = target_bytes(1.0, 1.0, 0, 0);
        let t = Target::from_bytes(&bytes);
        assert!((t.dist_m() - core::f32::consts::SQRT_2).abs() < 1e-5);
        assert!((t.angle_deg() - 45.0).abs() < 1e-4);
    }

    #[test]
    fn target_is_valid() {
        let zero = Target::default();
        assert!(!zero.is_valid());

        let bytes = target_bytes(0.5, 1.2, 0, 0);
        assert!(Target::from_bytes(&bytes).is_valid());
    }

    #[test]
    fn point_cloud_single_target() {
        let mut data = vec![0u8; 4 + 16];
        // count = 1
        data[0..4].copy_from_slice(&1u32.to_le_bytes());
        // target at (0.5, 1.2)
        let tb = target_bytes(0.5, 1.2, 2, 1);
        data[4..20].copy_from_slice(&tb);

        let frame = PointCloudFrame::from_bytes(&data);
        assert_eq!(frame.count, 1);
        assert!(frame.targets[0].is_valid());
        assert_eq!(frame.active_targets().count(), 1);
    }

    #[test]
    fn point_cloud_zero_targets() {
        let data = 0u32.to_le_bytes();
        let frame = PointCloudFrame::from_bytes(&data);
        assert_eq!(frame.count, 0);
        assert_eq!(frame.active_targets().count(), 0);
    }

    #[test]
    fn point_cloud_caps_at_three() {
        // Report count=5 but only 3 should be parsed
        let mut data = vec![0u8; 4 + 5 * 16];
        data[0..4].copy_from_slice(&5u32.to_le_bytes());
        let frame = PointCloudFrame::from_bytes(&data);
        assert_eq!(frame.count, 3);
    }
}
