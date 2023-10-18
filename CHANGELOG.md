
# Change Log

## Sprint 1 release (2023-08-18)
### Added
- Create the repository
- Define the structure of the repository
- Add documents exported from Confluence for Sprint 1


## Sprint 2 release (2023-09-21)
### Added
#### Confluence
- Guidelines for setting up Development Envronment
- Ethical considerations
- Cybersecurity analysis

#### Functional & Technical
- Add Docker File for environment set up 
- Pop up AI-generated image, which is realted to the input word, within Child UI
- Add filtering & moderation measures to ensure kids-friendly and safe conversation with NAO
- Add Strugging lettter indetifing module (have not intergrated into main code)
  
### Modified
#### Confluence
- Personas modified (Student->Child)
- Modified the page layout
- Modified the Sprint2 plan and retrospectives
#### Functional & Technical
- Migrate boxjelly form ROS1 to ROS2 Humble


## Sprint 3 release (2023-10-20)
### Added
#### Functional & Technical
- Distinction between the production (deployement) and development environment
- Automatic generation of the words using ChatGPT, based on the child's interest
- Migration of the manager UI to React
- Migration of the child UI to React
- Link made between the robot's speech and movements using ChatGPT
- Integration of the struggling letter algorithm to the project
- Integration of the Google Speech to Text model (taken from Bluering) to the project


### Modified
#### Confluence
- Structure of the space:
    - merged all the pages related to requirements under a single tree
    - split the development tree in 3 parts: development, testing and deployment
#### Functional & Technical
- Integration of the robot's controller direclty in the cowriter's node - the Flask API is not needed anymore
- Addition of a .conf file to launch all the nodes at once in the same container
