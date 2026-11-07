#include <glad/gl.h>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <optional>
#include "mesh.h"
#include "shader.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float baseRoll      = 0.0f;
float shoulderPitch = 0.0f;
float elbowPitch    = 0.0f;
float wristPitch    = 0.0f;
float gripperClaw   = 0.0f;

// Restores all metrics to original default positions
void resetPose() {
    baseRoll      = 0.0f;
    shoulderPitch = 0.0f;
    elbowPitch    = 0.0f;
    wristPitch    = 0.0f;
    gripperClaw   = 0.0f;
}

// PROCEDURAL GEOMETRY ENGINE
// Sphere Generation
std::pair<std::vector<Vertex>, std::vector<unsigned int>> createSphereData(float radius, int sectors, int stacks, const glm::vec4& color) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI / 2.0f - i * M_PI / stacks;
        float y = radius * sinf(phi);
        float r_cos_phi = radius * cosf(phi);

        for (int j = 0; j <= sectors; ++j) {
            float theta = j * 2.0f * M_PI / sectors;
            Vertex v;
            v.position = glm::vec3(r_cos_phi * cosf(theta), y, r_cos_phi * sinf(theta));
            v.normal   = glm::normalize(v.position); // Radial unit outward pointing vectors
            v.texCoord = glm::vec2((float)j / sectors, (float)i / stacks);
            v.color    = color;
            vertices.push_back(v);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        int k1 = i * (sectors + 1);
        int k2 = k1 + sectors + 1;
        for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
            if (i != 0) {
                indices.push_back(k1); indices.push_back(k2); indices.push_back(k1 + 1);
            }
            if (i != (stacks - 1)) {
                indices.push_back(k1 + 1); indices.push_back(k2); indices.push_back(k2 + 1);
            }
        }
    }
    return {vertices, indices};
}

// Cylinder Generation in Components
std::pair<std::vector<Vertex>, std::vector<unsigned int>> createCylinderData(float radius, float height, int sectors, const glm::vec4& color, int partitionMode) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int baseIndex = 0;

    // Outer Tube Walls
    if (partitionMode == 0 || partitionMode == 1) {
        for (int i = 0; i <= 1; ++i) {
            float y = (i == 0) ? 0.0f : height;
            float v = (i == 0) ? 0.0f : 1.0f;
            for (int j = 0; j <= sectors; ++j) {
                float theta = j * 2.0f * M_PI / sectors;
                Vertex vert;
                vert.position = glm::vec3(radius * cosf(theta), y, radius * sinf(theta));
                vert.normal   = glm::vec3(cosf(theta), 0.0f, sinf(theta));
                vert.texCoord = glm::vec2((float)j / sectors, v);
                vert.color    = color;
                vertices.push_back(vert);
            }
        }
        for (int j = 0; j < sectors; ++j) {
            unsigned int i0 = j, i1 = j + 1, i2 = j + (sectors + 1), i3 = i2 + 1;
            indices.push_back(i0); indices.push_back(i2); indices.push_back(i1);
            indices.push_back(i1); indices.push_back(i2); indices.push_back(i3);
        }
    }

    // Bottom End Cap
    if (partitionMode == 0 || partitionMode == 1) {
        baseIndex = static_cast<unsigned int>(vertices.size());
        Vertex bCenter; bCenter.position = glm::vec3(0, 0, 0); bCenter.normal = glm::vec3(0, -1, 0); bCenter.texCoord = glm::vec2(0.5f, 0.5f); bCenter.color = color;
        vertices.push_back(bCenter);
        for (int j = 0; j <= sectors; ++j) {
            float theta = j * 2.0f * M_PI / sectors;
            Vertex vert; vert.position = glm::vec3(radius * cosf(theta), 0.0f, radius * sinf(theta));
            vert.normal = glm::vec3(0, -1, 0); vert.texCoord = glm::vec2(0.5f + 0.5f * cosf(theta), 0.5f + 0.5f * sinf(theta)); vert.color = color;
            vertices.push_back(vert);
        }
        for (int j = 0; j < sectors; ++j) {
            indices.push_back(baseIndex); indices.push_back(baseIndex + 1 + j + 1); indices.push_back(baseIndex + 1 + j);
        }
    }

    // Top End Cap
    if (partitionMode == 0 || partitionMode == 2) {
        baseIndex = static_cast<unsigned int>(vertices.size());
        Vertex tCenter; tCenter.position = glm::vec3(0, height, 0); tCenter.normal = glm::vec3(0, 1, 0); tCenter.texCoord = glm::vec2(0.5f, 0.5f); tCenter.color = color;
        vertices.push_back(tCenter);
        for (int j = 0; j <= sectors; ++j) {
            float theta = j * 2.0f * M_PI / sectors;
            Vertex vert; vert.position = glm::vec3(radius * cosf(theta), height, radius * sinf(theta));
            vert.normal = glm::vec3(0, 1, 0); vert.texCoord = glm::vec2(0.5f + 0.5f * cosf(theta), 0.5f + 0.5f * sinf(theta)); vert.color = color;
            vertices.push_back(vert);
        }
        for (int j = 0; j < sectors; ++j) {
            indices.push_back(baseIndex); indices.push_back(baseIndex + 1 + j); indices.push_back(baseIndex + 1 + j + 1);
        }
    }

    return {vertices, indices};
}

// Cube Generation
std::pair<std::vector<Vertex>, std::vector<unsigned int>> createCuboidData(float dx, float dy, float dz, const glm::vec4& color) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    float x = dx / 2.0f, y = dy / 2.0f, z = dz / 2.0f;

    std::vector<glm::vec3> pos = {
        {-x,-y,-z}, { x,-y,-z}, { x, y,-z}, {-x, y,-z},
        {-x,-y, z}, { x,-y, z}, { x, y, z}, {-x, y, z}
    };
    std::vector<std::vector<int>> faceIndices = {
        {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 4, 7, 3}, {1, 2, 6, 5}
    };
    std::vector<glm::vec3> normals = {
        {0,0,-1}, {0,0,1}, {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}
    };

    unsigned int vIdx = 0;
    for (int i = 0; i < 6; ++i) {
        glm::vec2 uvs[4] = { {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f} };
        for (int j = 0; j < 4; ++j) {
            Vertex v;
            v.position = pos[faceIndices[i][j]];
            v.normal   = normals[i]; // Shared flat planar face normals
            v.texCoord = uvs[j];
            v.color    = color;
            vertices.push_back(v);
        }
        indices.push_back(vIdx);   indices.push_back(vIdx+1); indices.push_back(vIdx+2);
        indices.push_back(vIdx);   indices.push_back(vIdx+2); indices.push_back(vIdx+3);
        vIdx += 4;
    }
    return {vertices, indices};
}

// Checks multiple paths
std::string locateTexturePath(const std::string& filename) {
    std::vector<std::string> testPaths = {
        "../assets/" + filename,
        "../../assets/" + filename,
        "assets/" + filename
    };
    for (const auto& path : testPaths) {
        std::ifstream f(path);
        if (f.good()) return path;
    }
    return "";
}

// Main Running Function
int main() {
    sf::ContextSettings settings;
    settings.depthBits = 24; settings.stencilBits = 8; settings.antiAliasingLevel = 4;
    settings.majorVersion = 4; settings.minorVersion = 1;
    settings.attributeFlags = sf::ContextSettings::Core;

    sf::Window window(sf::VideoMode({1024, 768}), "Procedural 3D Robotic Arm Kinematics & Lighting", sf::State::Windowed, settings);
    window.setFramerateLimit(60);

    if (!gladLoadGL((GLADloadfunc)sf::Context::getFunction)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);

    // Shader files
    std::string vsPath = "shaders/mesh.vert";
    std::string fsPath = "shaders/mesh.frag";

    Material jointMaterial{ glm::vec3(0.2f), glm::vec3(0.9f, 0.7f, 0.1f), glm::vec3(0.4f), 32.0f, true };
    Material linkageMaterial{ glm::vec3(0.2f), glm::vec3(1.0f), glm::vec3(0.3f), 32.0f, true };
    Material metallicMaterial{ glm::vec3(0.2f), glm::vec3(0.4f, 0.4f, 0.45f), glm::vec3(0.5f), 16.0f, true };

    // Resource texture files
    std::string baseTexFile = locateTexturePath("texture_base_top.png");
    std::string armTexFile  = locateTexturePath("texture_arm_linkage.png");

    // Generate Procedural Components
    auto [bWallV, bWallI] = createCylinderData(1.10f, 0.70f, 40, glm::vec4(0.35f, 0.35f, 0.38f, 1.0f), 1);
    auto [bCapV, bCapI]   = createCylinderData(1.10f, 0.70f, 40, glm::vec4(1.0f), 2);
    auto [shSphV, shSphI] = createSphereData(0.24f, 24, 24, glm::vec4(0.0f, 0.8f, 0.4f, 1.0f));
    auto [uArmV, uArmI]   = createCylinderData(0.22f, 2.80f, 24, glm::vec4(1.0f), 0);
    auto [elSphV, elSphI] = createSphereData(0.20f, 24, 24, glm::vec4(0.0f, 0.8f, 0.4f, 1.0f));
    auto [fArmV, fArmI]   = createCylinderData(0.17f, 2.25f, 24, glm::vec4(1.0f), 0);
    auto [wrSphV, wrSphI] = createSphereData(0.17f, 24, 24, glm::vec4(0.0f, 0.8f, 0.4f, 1.0f));
    auto [plmV, plmI]     = createCuboidData(0.36f, 0.82f, 0.68f, glm::vec4(0.4f, 0.4f, 0.45f, 1.0f));
    auto [stmV, stmI]     = createCylinderData(0.045f, 0.58f, 12, glm::vec4(0.6f, 0.6f, 0.6f, 1.0f), 0);
    auto [fngV, fngI]     = createCuboidData(0.085f, 0.80f, 0.07f, glm::vec4(0.1f, 0.1f, 0.12f, 1.0f));
    
    // Set View Projections
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1024.0f / 768.0f, 0.1f, 100.0f);
    glm::mat4 view       = glm::lookAt(glm::vec3(9.5f, 6.0f, 11.5f), glm::vec3(0.0f, 2.5f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    
    
    bool running = true;
    while (running) {
    while (const std::optional<sf::Event> event = window.pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
    running = false;
    }
    if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
    if (keyPressed->code == sf::Keyboard::Key::Escape) {
    running = false;
    }
    if (keyPressed->code == sf::Keyboard::Key::Space) {
    resetPose();
    }}}
        
    // Active Control Inputs
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))  baseRoll += 2.0f;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left)) baseRoll -= 2.0f;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) shoulderPitch = glm::clamp(shoulderPitch + 1.5f, -10.0f, 95.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) shoulderPitch = glm::clamp(shoulderPitch - 1.5f, -10.0f, 95.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::F)) elbowPitch    = glm::clamp(elbowPitch + 1.5f, -45.0f, 135.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::R)) elbowPitch    = glm::clamp(elbowPitch - 1.5f, -45.0f, 135.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::G)) wristPitch    = glm::clamp(wristPitch + 1.5f, -90.0f, 90.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::T)) wristPitch    = glm::clamp(wristPitch - 1.5f, -90.0f, 90.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::O)) gripperClaw   = glm::clamp(gripperClaw + 0.01f, -0.15f, 1.20f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) gripperClaw   = glm::clamp(gripperClaw - 0.01f, -0.15f, 1.20f);
    glClearColor(0.11f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        
    // KINEMATIC TREE TRANSFORMATION STACK
    glm::mat4 accumulatedMatrix = glm::mat4(1.0f);
    
    // Base Roll
    accumulatedMatrix = glm::rotate(accumulatedMatrix, glm::radians(baseRoll), glm::vec3(0, 1, 0));
    Mesh baseWallsMesh(bWallV, bWallI, vsPath, fsPath, "", linkageMaterial);
    Mesh baseTopCapMesh(bCapV, bCapI, vsPath, fsPath, baseTexFile, linkageMaterial);
    baseWallsMesh.applyTransform(accumulatedMatrix);
    baseTopCapMesh.applyTransform(accumulatedMatrix);
    baseWallsMesh.draw(view, projection);
    baseTopCapMesh.draw(view, projection);
    // Advance up to shoulder height
    accumulatedMatrix = glm::translate(accumulatedMatrix, glm::vec3(0.0f, 0.70f, 0.0f));
    
    // Shoulder Pitch
    accumulatedMatrix = glm::rotate(accumulatedMatrix, glm::radians(shoulderPitch), glm::vec3(1, 0, 0));
    Mesh shoulderJointMesh(shSphV, shSphI, vsPath, fsPath, "", jointMaterial);
    Mesh upperArmLinkMesh(uArmV, uArmI, vsPath, fsPath, armTexFile, linkageMaterial);
    shoulderJointMesh.applyTransform(accumulatedMatrix);
    upperArmLinkMesh.applyTransform(accumulatedMatrix);
    shoulderJointMesh.draw(view, projection);
    upperArmLinkMesh.draw(view, projection);
    // Advance along Upper Arm cylinder length axis
    accumulatedMatrix = glm::translate(accumulatedMatrix, glm::vec3(0.0f, 2.80f, 0.0f));
    
    // Elbow Pitch
    accumulatedMatrix = glm::rotate(accumulatedMatrix, glm::radians(elbowPitch), glm::vec3(1, 0, 0));
    Mesh elbowJointMesh(elSphV, elSphI, vsPath, fsPath, "", jointMaterial);
    Mesh forearmLinkMesh(fArmV, fArmI, vsPath, fsPath, armTexFile, linkageMaterial);
    elbowJointMesh.applyTransform(accumulatedMatrix);
    forearmLinkMesh.applyTransform(accumulatedMatrix);
    elbowJointMesh.draw(view, projection);
    forearmLinkMesh.draw(view, projection);
    // Advance along Forearm cylinder length axis
    accumulatedMatrix = glm::translate(accumulatedMatrix, glm::vec3(0.0f, 2.25f, 0.0f));
    
    // Wrist Pitch
    accumulatedMatrix = glm::rotate(accumulatedMatrix, glm::radians(wristPitch), glm::vec3(1, 0, 0));
    Mesh wristJointMesh(wrSphV, wrSphI, vsPath, fsPath, "", jointMaterial);
    wristJointMesh.applyTransform(accumulatedMatrix);
    wristJointMesh.draw(view, projection);
    // Position Palm box offset
    glm::mat4 palmMatrix = glm::translate(accumulatedMatrix, glm::vec3(0.0f, 0.41f, 0.0f));
    Mesh palmBlockMesh(plmV, plmI, vsPath, fsPath, "", metallicMaterial);
    palmBlockMesh.applyTransform(palmMatrix);
    palmBlockMesh.draw(view, projection);
    // Split palm endpoint into symmetrical gripper wings
    glm::mat4 palmEndMatrix = glm::translate(accumulatedMatrix, glm::vec3(0.0f, 0.82f, 0.0f));
    
    // Symmetrical Left Claw
    glm::mat4 leftClawMatrix = glm::translate(palmEndMatrix, glm::vec3(-0.14f, 0.0f, 0.0f));
    Mesh leftStemMesh(stmV, stmI, vsPath, fsPath, "", metallicMaterial);
    leftStemMesh.applyTransform(leftClawMatrix);
    leftStemMesh.draw(view, projection);
    leftClawMatrix = glm::translate(leftClawMatrix, glm::vec3(0.0f, 0.58f, 0.0f));
    leftClawMatrix = glm::rotate(leftClawMatrix, glm::radians(gripperClaw * 55.0f), glm::vec3(0, 0, 1));
    glm::mat4 leftFingerMatrix = glm::translate(leftClawMatrix, glm::vec3(0.0f, 0.40f, 0.0f));
    Mesh leftFingerMesh(fngV, fngI, vsPath, fsPath, "", metallicMaterial);
    leftFingerMesh.applyTransform(leftFingerMatrix);
    leftFingerMesh.draw(view, projection);
    
    // Symmetrical Right Claw
    glm::mat4 rightClawMatrix = glm::translate(palmEndMatrix, glm::vec3(0.14f, 0.0f, 0.0f));
    Mesh rightStemMesh(stmV, stmI, vsPath, fsPath, "", metallicMaterial);
    rightStemMesh.applyTransform(rightClawMatrix);
    rightStemMesh.draw(view, projection);
    rightClawMatrix = glm::translate(rightClawMatrix, glm::vec3(0.0f, 0.58f, 0.0f));
    rightClawMatrix = glm::rotate(rightClawMatrix, glm::radians(-gripperClaw * 55.0f), glm::vec3(0, 0, 1));
    glm::mat4 rightFingerMatrix = glm::translate(rightClawMatrix, glm::vec3(0.0f, 0.40f, 0.0f));
    Mesh rightFingerMesh(fngV, fngI, vsPath, fsPath, "", metallicMaterial);
    rightFingerMesh.applyTransform(rightFingerMatrix);
    rightFingerMesh.draw(view, projection);
    window.display();
    }
    return 0;
    }
