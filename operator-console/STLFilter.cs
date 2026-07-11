using System;
using System.Collections.Generic;
using System.IO;

namespace 手术机器人上位机程序
{
    public struct Point3D
    {
        public float X, Y, Z;
        public Point3D(float x, float y, float z) { X = x; Y = y; Z = z; }
    }

    public struct Normal3D
    {
        public float Nx, Ny, Nz;
        public Normal3D(float x, float y, float z) { Nx = x; Ny = y; Nz = z; }
    }

    public sealed class Triangle
    {
        public Normal3D Normal { get; set; }
        public Point3D P1 { get; set; }
        public Point3D P2 { get; set; }
        public Point3D P3 { get; set; }
    }

    public struct ModelFeature
    {
        public double modelScale;
    }

    /// <summary>Minimal binary STL reader used by the legacy OpenGL robot view.</summary>
    public static class STLFilter
    {
        public static List<Triangle> LoadFromBINFile(string path)
        {
            var triangles = new List<Triangle>();
            if (!File.Exists(path)) return triangles;

            using (var reader = new BinaryReader(File.OpenRead(path)))
            {
                if (reader.BaseStream.Length < 84) return triangles;
                reader.ReadBytes(80);
                uint count = reader.ReadUInt32();
                if (84L + count * 50L > reader.BaseStream.Length)
                    throw new InvalidDataException("Invalid binary STL: " + path);

                for (uint i = 0; i < count; i++)
                {
                    var t = new Triangle
                    {
                        Normal = ReadNormal(reader),
                        P1 = ReadPoint(reader),
                        P2 = ReadPoint(reader),
                        P3 = ReadPoint(reader)
                    };
                    reader.ReadUInt16();
                    triangles.Add(t);
                }
            }
            return triangles;
        }

        public static ModelFeature GetModelFeature(IList<Triangle> triangles)
        {
            if (triangles == null || triangles.Count == 0)
                return new ModelFeature { modelScale = 1.0 };

            float minX = float.MaxValue, minY = float.MaxValue, minZ = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue, maxZ = float.MinValue;
            foreach (Triangle triangle in triangles)
            {
                Update(triangle.P1, ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
                Update(triangle.P2, ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
                Update(triangle.P3, ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            }
            return new ModelFeature { modelScale = Math.Max(maxX - minX, Math.Max(maxY - minY, maxZ - minZ)) };
        }

        private static Point3D ReadPoint(BinaryReader r) => new Point3D(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        private static Normal3D ReadNormal(BinaryReader r) => new Normal3D(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        private static void Update(Point3D p, ref float minX, ref float minY, ref float minZ, ref float maxX, ref float maxY, ref float maxZ)
        {
            minX = Math.Min(minX, p.X); minY = Math.Min(minY, p.Y); minZ = Math.Min(minZ, p.Z);
            maxX = Math.Max(maxX, p.X); maxY = Math.Max(maxY, p.Y); maxZ = Math.Max(maxZ, p.Z);
        }
    }
}

