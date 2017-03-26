//much of this code is taken from http://www.opengl-tutorial.org/


// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <string.h>
#include <iostream>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "controls.hpp"
#include "shader.hpp"

using namespace glm;
using namespace std;

int loadSTL(const char * path, std::vector<glm::vec3> & out_vertices, std::vector<glm::vec3> & out_normals) {
    ifstream file;                                      // Our file handler
    char *header = new char[80];                        // The 80-char file header
    unsigned int size = 0;								// The number of triangles in the file
    
    file.open(path, std::ios::in | std::ios::binary);
    
    if (file.is_open()) {            // Check that we opened successfully
        file.read(header, 80);							// Get the header
        file.read((char*)&size, 4);						// Get the number of triangles
        
        for (unsigned int i = 0; i < size; ++i) {		// Loop through all triangles
            float points[12] = { };						// 4 vectors * 3 points = 12 points
            short abc;									// Stores the attribute byte count
            
            for (unsigned int j = 0; j < 12; ++j) {		// Get all points from file
                file.read((char*)(points + j), 4);
                points[j] = floor((points[j] * 1000) + 0.5);    // TODO: why are we adding 0.5 again?
                // I remember now! It's an easy way to round up for >= 0.5, and down at < 0.5
            }
            file.read((char*)&abc, 2);
            
            for (int i = 0; i < 3; i++) { //has to be done for each vertex
                out_normals.push_back(glm::normalize(glm::vec3(points[0], points[1], points[2])));           // Create normal vector
            }
            
            double scale = 10000000;
            printf("points[3]: %d\n", points[3]);
            out_vertices.push_back(glm::vec3(points[3] / scale, points[4] / scale, points[5] / scale));	// Get first point of triangle
            out_vertices.push_back(glm::vec3(points[6] / scale, points[7] / scale, points[8] / scale));	// Get second point of triangle
            out_vertices.push_back(glm::vec3(points[9] / scale, points[10] / scale, points[11] / scale));	// Get third point of triangle
        }
        
        file.close();	// Close the file
    } else {
        printf("[ERROR] could not open file %s\n", path);
    }
    
    return size;
}

int main(int argc, char **argv) {
    
    if (argc <= 1) {
        printf("[ERROR] must provide at least one STL file\n");
        exit(0);
    }
    
    //load STLs
    std::vector<int> sizes;
    std::vector<vec3> vertices;
    std::vector<vec3> normals;
    
    for (int i = 1; i < argc; i++) {
        sizes.push_back(3 * loadSTL(argv[i], vertices, normals));
//        sizes.push_back(3 * loadSTL("../output_decomp_0.STL", vertices, normals));
//        sizes.push_back(3 * loadSTL("../output_decomp_1.STL", vertices, normals));
//        sizes.push_back(3 * loadSTL("../output_decomp_2.STL", vertices, normals));
    }
    
    // Initialise GLFW
    if (!glfwInit()) {
        fprintf(stderr, "[ERROR] failed to initialize GLFW\n");
        getchar();
        return -1;
    }
    
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1024, 768, "5AxLerViewer", nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr, "[ERROR] failed to open GLFW window\n");
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    
    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "[ERROR] failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }
    
    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 1024/2, 768/2);
    
    // White background
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    
    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);
    
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    
    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "../StandardShading.vertexshader", "../StandardShading.fragmentshader" );
    
    // Get a handle for our "MVP" uniform
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");
    GLuint ViewMatrixID = glGetUniformLocation(programID, "V");
    GLuint ModelMatrixID = glGetUniformLocation(programID, "M");
    
    //set up buffers
    
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);
    
    GLuint normalbuffer;
    glGenBuffers(1, &normalbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);
    
    
    // Get a handle for our "LightPosition" uniform
    glUseProgram(programID);
    GLuint LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
    GLuint ColorID = glGetUniformLocation(programID, "ObjectColor");
    
    do {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Use our shader
        glUseProgram(programID);
        
        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs(window);
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        
        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
        
        glm::vec3 lightPos = getPosition();//glm::vec3(5, 5, 5);
        glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);
        
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                              0,                  // attribute
                              3,                  // size
                              GL_FLOAT,           // type
                              GL_FALSE,           // normalized?
                              0,                  // stride
                              (void*)0            // array buffer offset
                              );
        
        // 2nd attribute buffer : normals
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
        glVertexAttribPointer(
                              1,                                // attribute
                              3,                                // size
                              GL_FLOAT,                         // type
                              GL_FALSE,                         // normalized?
                              0,                                // stride
                              (void*)0                          // array buffer offset
                              );
        
        
        int meshIndex = getMeshIndex();
        int meshIndexAdjusted = std::fmin(std::fmax(meshIndex, 0), sizes.size());
        
        // Draw the triangles !
        for (int i = 0; i < meshIndex; i++) {
            int start = 0;
            for (int j = 0; j < i; j++) {
                start += sizes[j];
            }
            glUniform3f(ColorID, 0, 1, (i == meshIndex - 1) ? 0 : 1);
            glDrawArrays(GL_TRIANGLES, start, sizes[i]);
        }
        
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        
        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
        
    } // Check if the ESC key was pressed or the window was closed
    while (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
          glfwWindowShouldClose(window) == 0);
    
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &normalbuffer);
    glDeleteProgram(programID);
    glDeleteVertexArrays(1, &VertexArrayID);
    
    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    
    return 0;
}