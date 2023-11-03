# NA-Redback - ChatGPT and NAO Robot

## Project Context

<p align="center">
  <img src="./docs/images/main_read_me_illustration.jpeg" />
</p>

Composed of a set of ROS nodes that facilitate the user interaction with a robot, the CoWriter project is designed for children to teach a social robot handwriting. The intuition behind this interaction is "learning by teaching" where children can have the acquisiton of handwriting by teaching a robot to write better. Nao is an autonomous, programmable humanoid robot and has been used for demonstration purposes for this project. 

Last semester, three different teams from the Software Project subject (COMP90082) took over this project with goals to:
1. update the CoWriter project from Python2 to Python3 ;
2. integrate ChatGPT to enable NAO robot to have conversations with children.

Currently, these projects are each deployed in different versions and environments and have achieved slightly different outcomes.

## Goals

This semester, our team aims to:
1. merge the previous projects in one unified environment ;
2. update the CoWriter learning algorithm and UI ;
3. enhance NAO's conversation capabilities using ChatGPT.

## Contributors

| Name | Role | Contact |
| ----------- | ----------- | ----------- |
| Wafa Johal | Client | wafa.johal@unimelb.edu.au |
| Sebastian Bobadilla | Project Supervisor | bobadillacha@unimelb.edu.au |
| Eunji Kim (Rachel) | Product Owner, Dev Member | kimek@student.unimelb.edu.au |
| Difan Wu | Scrum Master, Dev Member | difan.wu@student.unimelb.edu.au |
| Aurélien Plaire | Tech Lead | aplaire@student.unimelb.edu.au |
| Chien-Pu Lin (Jeff) | Test Lead | chienpu.lin@student.unimelb.edu.au
| Yangchen Shen | Quality Lead | yangchen.shen1@student.unimelb.edu.au |

## Structure

```
    ├── .github/workflows/              # Implementation of the GitHub Actions
    ├── docker/                         # Docker resources
    ├── docs/                           # Documentation files 
    ├── src/
    ├── .gitignore 
    ├── CHANGELOG.md
    ├── docker-compose.development.yml  # Docker compose file for development
    ├── docker-compose.production.yml   # Docker compose file for production
    ├── CHANGELOG.md
    ├── Makefile                        # Commands to launch the project
    ├── README.md
    └── requirements.txt                # Python dependencies
```

## Coding Standards

The Git branching model used for this project will be [Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

The general idea of this workflow is to work with 2 principal branches:

- *main* which will host the tested and finalised source code ;
- *develop* which will serve as a feature integration branch to test new releases before merging them on main.

More information about this workflow is available [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).


Most of our code development will be Python, so the coding standards used for this language will be the following:

- [PEP8](https://peps.python.org/pep-0008/) for all the coding conventions (ex: syntax checking, best practices) ;
- [Black](https://pypi.org/project/black/) for code formatting automation (ex: maximum character number per line).

To ensure that the code pushed is consistent with the chosen Python standards, a [GitHub Action](https://github.com/features/actions) has been set on the repository (see the `.github/workflows` folder).

At each commit, a pipeline will be triggered. It will execute a command (module [Flake8](https://pypi.org/project/flake8/)) to check if each specified Python file complies with PEP8’s rules.

If the pipeline should fail, it would mean some changes have to be made to ensure the consistency of the source code.

## Sprint 2 Demo Video
The demo video presenting the features developed during Sprint 2 is available [here](https://drive.google.com/file/d/13TsE_G87LoL3ysXPLU3SCg8_5_rH3eh1/view?usp=drive_link).

## Sprint 3 Demo Video
The demo video presenting the features developed during Sprint 3 is available [here](https://drive.google.com/file/d/1dBI04GYDs1uH_ORdyimv_hFUaKAXvo5X/view?usp=sharing).

## Sprint 4 Final Video
The final video of our poduct is available [here](https://youtu.be/7Zr4GBAgWPE).
