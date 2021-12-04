# proc_underwater_com

![Docker Image CI - Master Branch](https://github.com/sonia-auv/proc_underwater_com/workflows/Docker%20Image%20CI%20-%20Master%20Branch/badge.svg)
![Docker Image CI - Develop Branch](https://github.com/sonia-auv/proc_underwater_com/workflows/Docker%20Image%20CI%20-%20Develop%20Branch/badge.svg?branch=develop)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/sonia-auv/proc_underwater_com)
![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/sonia-auv/proc_underwater_com.svg)


*Please read the instructions and fill in the blanks*


One Paragraph of project description goes here

## Getting Started

Clone current project by using following command :
```bash
    git clone git@github.com:sonia-auv/proc_underwater_com.git
```

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

**IMPORTANT :** *If you have just imported your repository, please follow the instructions in* [BOOTSTRAP.md](BOOTSTRAP.md) (Once the bootstrap completed, you can remove this comment from the README)

### Prerequisites

First and foremost to run the module you will need to have [docker](https://www.docker.com/get-started?utm_source=google&utm_medium=cpc&utm_campaign=getstarted&utm_content=sitelink&utm_term=getstarted&utm_budget=growth&gclid=CjwKCAjw57b3BRBlEiwA1Imytuv9VRFX5Z0INBaD3JJNSUmadgQh7ZYWTw_r-yFn2S4XjZTsLbNnnBoCPsIQAvD_BwE) installed.

To validate your installation of docker, simply type in

```
docker -v
```

If you receive an output in the likes of :
```
Docker version 19.03.5, build 633a0ea
```

It means you have it installed. If not follow instructions on how to install it for your OS.


## Deployment

To use the mission array to control both submarine with the same state machine, you can send the right state value with the appropriate mission ID. Here is a link to the documentation to explain the functionnality :

*[Underwater COM documentation](https://wiki.sonia.etsmtl.ca/en/software/projects/underwater-com) - Special project documentation with mission array explained

## Built With

Add additional project dependencies

* [ROS](http://wiki.ros.org/) - ROS robotic framework


## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
