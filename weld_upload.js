(function () {
    let cameraRigGroup = null;
    let uploadedPlyBuffer = null;
    let uploadedPlyName = '';
    const calibrationCameras = [
        { name: 'Depth / world', position: [0, 0, 0], color: 0x00d2ff },
        { name: 'IR left', position: [0, 0, 0], color: 0xffffff },
        { name: 'IR right', position: [-0.17864432, -0.000386631, 0.009129403], color: 0xffb300 }
    ];
    window.initUploadTools = function () {
        bindUploadUi();
        createCameraRig();
        updateCameraReadout(null);
    };
    function bindUploadUi() {
        const input = document.getElementById('ply-upload');
        const detectButton = document.getElementById('btn-detect-upload');
        const demoButton = document.getElementById('btn-use-demo');
        const cameraToggle = document.getElementById('toggle-camera-rig');
        if (input) {
            input.addEventListener('change', async (event) => {
                const file = event.target.files && event.target.files[0];
                if (!file) return;
                uploadedPlyName = file.name;
                uploadedPlyBuffer = await file.arrayBuffer();
                setUploadStatus('已载入 ' + file.name + '，点击运行检测');
            });
        }
        if (detectButton) detectButton.addEventListener('click', runUploadedDetection);
        if (demoButton) {
            demoButton.addEventListener('click', () => {
                plateCoords = WELD_DATA.plates.slice();
                gapCoords = WELD_DATA.boundaries.slice();
                trajCoords = WELD_DATA.trajectory.slice();
                refreshScene(WELD_DATA.stats, null);
                setUploadStatus('已恢复内置示例数据');
            });
        }
        if (cameraToggle) {
            cameraToggle.addEventListener('change', (event) => {
                if (cameraRigGroup) cameraRigGroup.visible = event.target.checked;
            });
        }
    }
    async function runUploadedDetection() {
        if (!uploadedPlyBuffer) {
            setUploadStatus('请先选择 PLY 文件');
            return;
        }
        try {
            setUploadStatus('正在解析 PLY 并检测焊缝... 大点云可能需要几秒');
            await new Promise((resolve) => setTimeout(resolve, 30));
            const parsed = parsePlyWithThree(uploadedPlyBuffer);
            const plyCamera = parsePlyCamera(uploadedPlyBuffer);
            const result = detectWeldSeam(parsed.points, parsed.totalPoints);
            plateCoords = result.plates;
            gapCoords = result.boundaries;
            trajCoords = result.trajectory;
            refreshScene(result.stats, plyCamera);
            setUploadStatus('完成：' + uploadedPlyName + '，检测到 ' + trajCoords.length + ' 个轨迹点');
        } catch (error) {
            console.error(error);
            setUploadStatus('检测失败：' + error.message);
        }
    }
    function parsePlyWithThree(buffer) {
        if (!THREE.PLYLoader) throw new Error('PLYLoader 未加载');
        const geometry = new THREE.PLYLoader().parse(buffer);
        const attr = geometry.getAttribute('position');
        if (!attr || attr.count === 0) throw new Error('PLY 中没有 position 点坐标');
        const step = Math.max(1, Math.ceil(attr.count / 360000));
        const points = [];
        for (let i = 0; i < attr.count; i += step) {
            points.push([attr.getX(i), attr.getY(i), attr.getZ(i)]);
        }
        geometry.dispose();
        return { points, totalPoints: attr.count };
    }
    function parsePlyCamera(buffer) {
        const bytes = new Uint8Array(buffer);
        const marker = 'end_header';
        let headerEnd = -1;
        for (let i = 0; i < Math.min(bytes.length - marker.length, 20000); i++) {
            let matched = true;
            for (let j = 0; j < marker.length; j++) {
                if (bytes[i + j] !== marker.charCodeAt(j)) { matched = false; break; }
            }
            if (matched) {
                let k = i + marker.length;
                while (k < bytes.length && (bytes[k] === 13 || bytes[k] === 10)) k++;
                headerEnd = k;
                break;
            }
        }
        if (headerEnd < 0) return null;
        const header = new TextDecoder().decode(bytes.slice(0, headerEnd));
        const lines = header.split(/\r?\n/);
        const formatLine = lines.find((line) => line.startsWith('format ')) || '';
        if (formatLine.split(/\s+/)[1] !== 'binary_little_endian') return null;
        let active = '';
        let vertexCount = 0;
        let cameraCount = 0;
        const vertexProps = [];
        const cameraProps = [];
        for (const line of lines) {
            const parts = line.trim().split(/\s+/);
            if (parts[0] === 'element') {
                active = parts[1];
                if (active === 'vertex') vertexCount = parseInt(parts[2], 10);
                if (active === 'camera') cameraCount = parseInt(parts[2], 10);
            } else if (parts[0] === 'property') {
                if (active === 'vertex') vertexProps.push({ type: parts[1], name: parts[2] });
                if (active === 'camera') cameraProps.push({ type: parts[1], name: parts[2] });
            }
        }
        if (!vertexCount || !cameraCount || cameraProps.length === 0) return null;
        const view = new DataView(buffer);
        const vertexStride = vertexProps.reduce((sum, prop) => sum + plyTypeSize(prop.type), 0);
        let offset = headerEnd + vertexCount * vertexStride;
        const values = {};
        cameraProps.forEach((prop) => {
            values[prop.name] = readPlyValue(view, offset, prop.type);
            offset += plyTypeSize(prop.type);
        });
        if (!Number.isFinite(values.view_px) || !Number.isFinite(values.view_py) || !Number.isFinite(values.view_pz)) return null;
        return {
            name: 'PLY capture',
            position: [values.view_px, values.view_py, values.view_pz],
            axes: [
                [values.x_axisx || 1, values.x_axisy || 0, values.x_axisz || 0],
                [values.y_axisx || 0, values.y_axisy || 1, values.y_axisz || 0],
                [values.z_axisx || 0, values.z_axisy || 0, values.z_axisz || 1]
            ],
            color: 0x8a5cff
        };
    }
    function plyTypeSize(type) {
        return ({ char: 1, uchar: 1, int8: 1, uint8: 1, short: 2, ushort: 2, int16: 2, uint16: 2, int: 4, uint: 4, int32: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 })[type] || 4;
    }
    function readPlyValue(view, offset, type) {
        if (type === 'double' || type === 'float64') return view.getFloat64(offset, true);
        if (type === 'float' || type === 'float32') return view.getFloat32(offset, true);
        if (type === 'uchar' || type === 'uint8') return view.getUint8(offset);
        if (type === 'char' || type === 'int8') return view.getInt8(offset);
        if (type === 'ushort' || type === 'uint16') return view.getUint16(offset, true);
        if (type === 'short' || type === 'int16') return view.getInt16(offset, true);
        if (type === 'uint' || type === 'uint32') return view.getUint32(offset, true);
        return view.getInt32(offset, true);
    }
    function detectWeldSeam(points, totalPoints) {
        const downsampled = voxelDownsample(points, 0.001);
        const plane = fitPlaneZ(downsampled);
        const bounds = getPointBounds(downsampled);
        if (!bounds) throw new Error('点云为空');
        const sliceCount = 150;
        const ySpan = bounds.maxY - bounds.minY || 1;
        const bins = Array.from({ length: sliceCount }, () => []);
        downsampled.forEach((point) => {
            const idx = Math.min(sliceCount - 1, Math.max(0, Math.floor((point[1] - bounds.minY) / ySpan * sliceCount)));
            bins[idx].push(point);
        });
        const centers = [];
        const boundaries = [];
        let lastX = null;
        for (let i = 0; i < sliceCount; i++) {
            const slice = bins[i];
            if (slice.length < 30) continue;
            slice.sort((a, b) => a[0] - b[0]);
            const candidates = [];
            for (let j = 0; j < slice.length - 1; j++) {
                const diff = slice[j + 1][0] - slice[j][0];
                if (!((diff >= 0.002 && diff <= 0.008) || (diff >= 0.0015 && diff <= 0.012))) continue;
                const xl = slice[j][0];
                const xr = slice[j + 1][0];
                let leftCount = 0;
                let rightCount = 0;
                for (let k = 0; k < slice.length; k++) {
                    const x = slice[k][0];
                    if (x > xl - 0.045 && x < xl - 0.005) leftCount++;
                    if (x > xr + 0.005 && x < xr + 0.045) rightCount++;
                }
                const requiredCount = Math.max(12, Math.min(60, Math.floor(slice.length * 0.04)));
                if (leftCount >= requiredCount && rightCount >= requiredCount) candidates.push(j);
            }
            if (candidates.length === 0) continue;
            let best = candidates[0];
            if (lastX === null) {
                const midX = (slice[0][0] + slice[slice.length - 1][0]) / 2;
                best = candidates.reduce((a, b) => Math.abs(centerX(slice, a) - midX) < Math.abs(centerX(slice, b) - midX) ? a : b);
            } else {
                best = candidates.reduce((a, b) => Math.abs(centerX(slice, a) - lastX) < Math.abs(centerX(slice, b) - lastX) ? a : b);
            }
            const p1 = slice[best];
            const p2 = slice[best + 1];
            const x = (p1[0] + p2[0]) / 2;
            const y = bounds.minY + (i + 0.5) * ySpan / sliceCount;
            const z = plane.ax * x + plane.by * y + plane.c;
            if (lastX === null || Math.abs(x - lastX) < 0.04) {
                centers.push([x, y, z]);
                boundaries.push(p1, p2);
                lastX = x;
            }
        }
        if (centers.length < 8) throw new Error('未检测到连续焊缝空隙，请确认点云坐标方向与钢板拼缝一致');
        const trajectory = smoothTrajectory(centers);
        return {
            plates: downsampled,
            boundaries,
            trajectory,
            stats: {
                total_points: totalPoints,
                seam_length: polylineLength(trajectory) * 1000,
                avg_width: averageGapWidth(boundaries) * 1000
            }
        };
    }
    function centerX(points, idx) {
        return (points[idx][0] + points[idx + 1][0]) / 2;
    }
    function voxelDownsample(points, voxel) {
        const map = new Map();
        points.forEach((point) => {
            const key = Math.round(point[0] / voxel) + ',' + Math.round(point[1] / voxel) + ',' + Math.round(point[2] / voxel);
            if (!map.has(key)) map.set(key, point);
        });
        return Array.from(map.values());
    }
    function fitPlaneZ(points) {
        let sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
        const n = points.length;
        for (const p of points) {
            sx += p[0]; sy += p[1]; sz += p[2];
            sxx += p[0] * p[0]; syy += p[1] * p[1]; sxy += p[0] * p[1];
            sxz += p[0] * p[2]; syz += p[1] * p[2];
        }
        const m = [[sxx, sxy, sx, sxz], [sxy, syy, sy, syz], [sx, sy, n, sz]];
        for (let i = 0; i < 3; i++) {
            let pivot = i;
            for (let r = i + 1; r < 3; r++) if (Math.abs(m[r][i]) > Math.abs(m[pivot][i])) pivot = r;
            const tmp = m[i]; m[i] = m[pivot]; m[pivot] = tmp;
            const div = m[i][i] || 1;
            for (let c = i; c < 4; c++) m[i][c] /= div;
            for (let r = 0; r < 3; r++) {
                if (r === i) continue;
                const f = m[r][i];
                for (let c = i; c < 4; c++) m[r][c] -= f * m[i][c];
            }
        }
        return { ax: m[0][3], by: m[1][3], c: m[2][3] };
    }
    function smoothTrajectory(points) {
        points.sort((a, b) => a[1] - b[1]);
        return points.map((_, i) => {
            let sx = 0, sy = 0, sz = 0, n = 0;
            for (let j = Math.max(0, i - 2); j <= Math.min(points.length - 1, i + 2); j++) {
                sx += points[j][0]; sy += points[j][1]; sz += points[j][2]; n++;
            }
            return [sx / n, sy / n, sz / n];
        });
    }
    function polylineLength(points) {
        let length = 0;
        for (let i = 1; i < points.length; i++) {
            const a = points[i - 1];
            const b = points[i];
            length += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
        }
        return length;
    }
    function averageGapWidth(boundaries) {
        if (!boundaries.length) return NaN;
        let sum = 0;
        let n = 0;
        for (let i = 0; i + 1 < boundaries.length; i += 2) {
            sum += Math.abs(boundaries[i + 1][0] - boundaries[i][0]);
            n++;
        }
        return sum / Math.max(1, n);
    }
    function refreshScene(stats, plyCamera) {
        isSimulating = false;
        simProgress = 0;
        disposeObject(platePoints);
        disposeObject(gapPoints);
        disposeObject(trajLine);
        disposeObject(torchMesh);
        disposeObject(sparkParticles);
        platePoints = gapPoints = trajLine = torchMesh = sparkParticles = null;
        renderPlates();
        renderGapBoundaries();
        renderTrajectory();
        createWeldingTorch();
        createSparkParticles();
        fitCameraToScene();
        updateStats(stats);
        createCameraRig(plyCamera);
    }
    function disposeObject(object) {
        if (!object) return;
        scene.remove(object);
        if (object.traverse) object.traverse(disposeMesh);
        else disposeMesh(object);
    }
    function disposeMesh(object) {
        if (object.geometry) object.geometry.dispose();
        if (object.material) {
            if (Array.isArray(object.material)) object.material.forEach((item) => item.dispose());
            else object.material.dispose();
        }
    }
    function updateStats(stats) {
        document.getElementById('stat-length').innerHTML = numberOrDash(stats.seam_length, 2) + ' <span class="stat-unit">mm</span>';
        document.getElementById('stat-width').innerHTML = numberOrDash(stats.avg_width, 2) + ' <span class="stat-unit">mm</span>';
        document.getElementById('stat-pts').innerHTML = (stats.total_points / 10000).toFixed(1) + ' <span class="stat-unit">万点</span>';
        document.getElementById('stat-time').innerHTML = Math.ceil((stats.seam_length || 0) / 5.0) + ' <span class="stat-unit">秒</span>';
    }
    function numberOrDash(value, digits) {
        return Number.isFinite(value) ? value.toFixed(digits) : '--';
    }
    function createCameraRig(plyCamera) {
        disposeObject(cameraRigGroup);
        cameraRigGroup = new THREE.Group();
        const bounds = getPointBounds(plateCoords);
        const origin = bounds ? new THREE.Vector3(bounds.minX, bounds.minY, bounds.minZ) : new THREE.Vector3(0, 0, 0);
        const span = bounds ? Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY, bounds.maxZ - bounds.minZ, 0.12) : 0.12;
        addWorldAxes(cameraRigGroup, origin, span * 0.18);
        calibrationCameras.forEach((cam) => addCameraMarker(cameraRigGroup, cam, span * 0.08));
        if (plyCamera) addCameraMarker(cameraRigGroup, plyCamera, span * 0.1);
        const toggle = document.getElementById('toggle-camera-rig');
        cameraRigGroup.visible = toggle ? toggle.checked : true;
        scene.add(cameraRigGroup);
        updateCameraReadout(plyCamera);
    }
    function getPointBounds(points) {
        if (!points || points.length === 0) return null;
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        for (const point of points) {
            if (point[0] < minX) minX = point[0];
            if (point[0] > maxX) maxX = point[0];
            if (point[1] < minY) minY = point[1];
            if (point[1] > maxY) maxY = point[1];
            if (point[2] < minZ) minZ = point[2];
            if (point[2] > maxZ) maxZ = point[2];
        }
        return { minX, minY, minZ, maxX, maxY, maxZ };
    }
    function addWorldAxes(group, origin, scale) {
        group.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, scale, 0xff3b30, scale * 0.22, scale * 0.08));
        group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, scale, 0x2ecc71, scale * 0.22, scale * 0.08));
        group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, scale, 0x00a0ff, scale * 0.22, scale * 0.08));
    }
    function addCameraMarker(group, cam, scale) {
        const pos = new THREE.Vector3(cam.position[0], cam.position[1], cam.position[2]);
        const color = cam.color || 0xffb300;
        const marker = new THREE.Mesh(new THREE.SphereGeometry(scale * 0.12, 12, 12), new THREE.MeshBasicMaterial({ color }));
        marker.position.copy(pos);
        group.add(marker);
        group.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), pos, scale, 0xff3b30, scale * 0.2, scale * 0.07));
        group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), pos, scale, 0x2ecc71, scale * 0.2, scale * 0.07));
        group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), pos, scale, 0x00a0ff, scale * 0.2, scale * 0.07));
    }
    function updateCameraReadout(plyCamera) {
        const el = document.getElementById('camera-readout');
        if (!el) return;
        const rows = ['XYZ: red=X green=Y blue=Z'];
        calibrationCameras.forEach((cam) => rows.push(cam.name + ': [' + cam.position.map((v) => v.toFixed(4)).join(', ') + '] m'));
        if (plyCamera) rows.push('PLY camera: [' + plyCamera.position.map((v) => v.toFixed(4)).join(', ') + '] m');
        rows.push('右相机由现有外参按米制估计；若采集软件定义为 world-to-camera，方向需取逆矩阵。');
        el.textContent = rows.join('\n');
    }
    function setUploadStatus(text) {
        const el = document.getElementById('upload-status');
        if (el) el.textContent = text;
    }
})();



