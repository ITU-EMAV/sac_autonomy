#!/usr/bin/env python3
"""PCD haritayi voxel merkezleri (centroid) ile seyreltir.

Neden: NDT haritayi get_differential_pointcloud_map SERVISINDEN ham olarak
aliyor; o yolda leaf parametresi YOK (map_loader'daki leaf_size sadece
RViz'in /map/downsampled_pointcloud_map topic'ine uygulanir). Dolayisiyla
NDT'ye giden haritayi seyreltmenin tek yolu PCD'nin kendisini seyreltmek.

ndt.resolution 2.5 iken haritada voxel basina ~28 nokta olmasi gereksiz;
Gauss kovaryansi icin min_points_per_voxel = 6 (kodda sabit) yetiyor.
Olculen (GlobalMap.pcd, res 2.5):
    ham       10.932.779 nokta -> 77.388 voxel, %74.0'i >=6 nokta, medyan 28
    leaf 0.5   1.762.391 nokta -> 77.376 voxel, %67.0'i >=6 nokta, medyan 13
    leaf 1.0     494.510 nokta -> 74.222 voxel, %50.1'i >=6 nokta, medyan  6
Voxel sayisi 0.5'te neredeyse hic degismiyor, nokta sayisi 6.2x duşuyor.

DIKKAT: ndt.resolution sonradan DUSURULURSE (or. 1.0) bu seyreltilmis
harita zayif kalir. Orijinal dosya silinmiyor, geri donulebilir.

Kullanim:
    python3 downsample_map.py <girdi.pcd> <cikti.pcd> <leaf_m>
"""
import sys

import numpy as np


def read_pcd_xyzi(path):
    with open(path, 'rb') as f:
        header = b''
        while b'\nDATA' not in header or not header.endswith(b'\n'):
            chunk = f.read(1)
            if not chunk:
                raise RuntimeError('PCD basligi bitmeden dosya bitti')
            header += chunk
            if header.endswith(b'binary\n') or header.endswith(b'ascii\n'):
                break
        offset = f.tell()
    text = header.decode('ascii', 'replace')
    if 'ascii' in text.split('DATA')[-1]:
        raise RuntimeError('Sadece binary PCD destekleniyor')
    fields = next(l.split()[1:] for l in text.splitlines() if l.startswith('FIELDS'))
    if fields[:3] != ['x', 'y', 'z']:
        raise RuntimeError(f'Beklenen FIELDS x y z ...; bulunan {fields}')
    data = np.fromfile(path, dtype=np.float32, offset=offset)
    ncol = len(fields)
    data = data[:(len(data) // ncol) * ncol].reshape(-1, ncol)
    return data, fields


def write_pcd(path, pts, fields):
    n = len(pts)
    head = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        f'FIELDS {" ".join(fields)}\n'
        f'SIZE {" ".join(["4"] * len(fields))}\n'
        f'TYPE {" ".join(["F"] * len(fields))}\n'
        f'COUNT {" ".join(["1"] * len(fields))}\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\nDATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(head.encode('ascii'))
        f.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 1
    src, dst, leaf = sys.argv[1], sys.argv[2], float(sys.argv[3])

    pts, fields = read_pcd_xyzi(src)
    pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
    print(f'girdi : {len(pts):,} nokta, alanlar {fields}')

    # Voxel indeksi -> centroid. PCL'in VoxelGrid'i de centroid kullanir;
    # ilk noktayi secmek yuzeyleri tirtikli birakir.
    keys = np.floor(pts[:, :3] / leaf).astype(np.int64)
    keys -= keys.min(axis=0)
    _, inverse = np.unique(
        keys[:, 0] * 4_000_000_000 + keys[:, 1] * 2_000_000 + keys[:, 2],
        return_inverse=True)
    counts = np.bincount(inverse).astype(np.float64)
    out = np.empty((len(counts), pts.shape[1]), dtype=np.float32)
    for c in range(pts.shape[1]):
        out[:, c] = np.bincount(inverse, weights=pts[:, c]) / counts

    write_pcd(dst, out, fields)
    print(f'cikti : {len(out):,} nokta  (leaf {leaf} m, {len(pts) / len(out):.1f}x azalma)')
    print(f'        {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
