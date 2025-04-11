using UnityEngine;

/// <summary>
/// Provides utility methods for generating mesh primitives at runtime.
/// </summary>
public class MeshGenerator : MonoBehaviour
{
    /// <summary>
    /// Creates a simple quad mesh with centered origin.
    /// </summary>
    /// <returns>A Mesh representing a quad with UVs and normals.</returns>
    public static Mesh CreateQuadMesh()
    {
        Mesh mesh = new Mesh();

        // Define four vertices for the quad
        Vector3[] vertices = new Vector3[4];
        vertices[0] = new Vector3(-0.5f, -0.5f, 0); // Bottom left
        vertices[1] = new Vector3(0.5f, -0.5f, 0);  // Bottom right
        vertices[2] = new Vector3(-0.5f, 0.5f, 0);  // Top left
        vertices[3] = new Vector3(0.5f, 0.5f, 0);   // Top right

        // Assign UV coordinates for texturing
        Vector2[] uv = new Vector2[4];
        uv[0] = new Vector2(0, 0);
        uv[1] = new Vector2(1, 0);
        uv[2] = new Vector2(0, 1);
        uv[3] = new Vector2(1, 1);

        // Define two triangles using the four vertices
        int[] triangles = new int[6];
        triangles[0] = 0;
        triangles[1] = 2;
        triangles[2] = 1;
        triangles[3] = 1;
        triangles[4] = 2;
        triangles[5] = 3;

        // Apply mesh attributes
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        return mesh;
    }
}
