//
//  controls.hpp
//  5AxLerV2
//
//  Created by Ethan Coeytaux on 3/22/17.
//  Copyright © 2017 mapmqp. All rights reserved.
//

#ifndef CONTROLS_HPP
#define CONTROLS_HPP

void computeMatricesFromInputs(GLFWwindow* window);
glm::vec3 getPosition();
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
int getMeshIndex();

#endif
