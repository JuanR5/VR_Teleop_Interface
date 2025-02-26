using UnityEngine;

public class MeshGenerator : MonoBehaviour
{
    public static Mesh CreateQuadMesh()
    {
        // Create a new mesh for the quad
        Mesh mesh = new Mesh();
        
        // Define the vertices of the quad
        Vector3[] vertices = new Vector3[4];
        vertices[0] = new Vector3(-0.5f, -0.5f, 0); // Bottom left
        vertices[1] = new Vector3(0.5f, -0.5f, 0);  // Bottom right
        vertices[2] = new Vector3(-0.5f, 0.5f, 0);  // Top left
        vertices[3] = new Vector3(0.5f, 0.5f, 0);   // Top right

        // Define the UVs (mapping of texture coordinates to the quad)
        Vector2[] uv = new Vector2[4];
        uv[0] = new Vector2(0, 0); // Bottom left
        uv[1] = new Vector2(1, 0); // Bottom right
        uv[2] = new Vector2(0, 1); // Top left
        uv[3] = new Vector2(1, 1); // Top right

        // Define the triangles (indices for drawing the quad)
        int[] triangles = new int[6];
        triangles[0] = 0; // Bottom left
        triangles[1] = 2; // Top left
        triangles[2] = 1; // Bottom right
        triangles[3] = 1; // Bottom right
        triangles[4] = 2; // Top left
        triangles[5] = 3; // Top right

        // Set the mesh data
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;

        // Recalculate normals for proper lighting and rendering
        mesh.RecalculateNormals();

        return mesh;
    }
}
