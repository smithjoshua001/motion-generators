# Motion Generators (WIP)

This package contains several motion generators for use with robots in joint space and cartesian space.

## Getting Started

The following instructions will show you how to get the following library installed.

### Prerequisites

The library is broken up into different sections to allow for different functionality. Each of these sections will require different libraries to function. They shall be listed here.


<div class="row" style="display:flex; justify-content: center; align-items: flex-start;">
    <div class="column" style="flex: 33.33%; padding: 5px;">

| Standalone Library |
|:----:|
| Eigen (3.3.4) |
| RapidJSON (1.1.0) |

</div>
<div class="column" style="flex: 33.33%; padding: 5px;">

| Python Bindings | 
|:----:|
| Same as Standalone|
| Swig |
| SwigMake |

</div>
<div class="column" style="flex: 33.33%; padding: 5px;">

| Orocos | 
|:----:|
| Same as Standalone |
| OROCOS (2.9) |
| RST-RT |

</div>
</div>


### Installing

The library uses CMake as the primary build tool. The steps to compile are as follows:

```
cd /path/to/motion-generators
mkdir build
cd build
cmake ..
make
(sudo) make install
```

<!-- ## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds



## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 


## Authors

* **Joshua Smith** - [smithjoshua001](https://github.com/smithjoshua001)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc

-->