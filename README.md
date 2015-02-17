# io-isense

Nim bindings for the InterSense SDK.
![io-isense Logo](logo.png)


## About

io-isense contains bindings to the InterSense SDK for the
[Nim](http://nim-lang.org) programming language. InterSense provides a number of
inertial, magnetic and GPS 3-dof and 6-dof position tracking devices, such as
the InertiaCube series and IS series systems.


## Supported Platforms

io-isense is still under heavy development and does not work yet. So far, the
following platforms have been built and tested:

- ~~Linux~~
- ~~Mac OSX~~
- Windows


## Prerequisites

TODO

### Linux

TODO

### Mac OSX

TODO

### Windows

Download the latest InterSense SDK or Library from the InterSense web site and copy the
contained ``isense.dll`` file into your program's folder.


## Dependencies

io-isense does not have any dependencies to other Nim packages at this time.


## Usage

Import the `isense` module from this package to make the bindings available in your project:

```Nimrod
import spnav
```

## Support

Please [file an issue](https://github.com/nimious/io-isense/issues), submit a
[pull request](https://github.com/nimious/io-isense/pulls?q=is%3Aopen+is%3Apr)
or email us at info@nimio.us if this package is out of date or contains bugs.
For all other issues related to Oculus devices or the device driver software
visit the Oculus web sites below.


## References

- [InterSense Homepage](https://www.intersense.com)
- [InterSense SDK Download Page](https://http://www.intersense.com/pages/33/154/)
