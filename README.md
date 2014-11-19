#ArduPilot Project#

### The ArduPilot project is made up of: ###
>>ArduCopter (or APM:Copter) : [code](https://github.com/diydrones/ardupilot/tree/master/ArduCopter), [wiki](http://copter.ardupilot.com)

>>ArduPlane (or APM:Plane) : [code](https://github.com/diydrones/ardupilot/tree/master/ArduPlane), [wiki](http://plane.ardupilot.com)

>>ArduRover (or APMrover2) : [code](https://github.com/diydrones/ardupilot/tree/master/APMrover2), [wiki](http://rover.ardupilot.com)

>>Antenna Tracker : [code](https://github.com/diydrones/ardupilot/tree/master/AntennaTracker), [wiki](http://copter.ardupilot.com/wiki/common-antennatracker-introduction)

### User Support & Discussion Forums ###
>>APM Forum: [http://ardupilot.com/forum/index.php](http://ardupilot.com/forum/index.php)

>>Community Site: [http://diydrones.com](http://diydrones.com)

### Developer Information ###
>>Github repository: [https://github.com/diydrones/ardupilot](https://github.com/diydrones/ardupilot)

>>Main developer wiki: [http://dev.ardupilot.com](http://dev.ardupilot.com)

>>Developer email group: drones-discuss@googlegroups.com

### Contributors ###
>>[Github statistics](https://github.com/diydrones/ardupilot/graphs/contributors)

### How To Get Involved ###
>>The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](http://dev.ardupilot.com/wiki/guidelines-for-contributors-to-the-apm-codebase)

>>We have an active group of Beta Testers especially for ArduCopter to help us find bugs: [release procedures](http://dev.ardupilot.com/wiki/release-procedures)

>>Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/diydrones/ardupilot/issues).

>>Helping other users with log analysis on [diydrones.com](http://www.diydrones.com) and the [APM forums ](http://ardupilot.com/forum/index.php) is always appreciated:

>>There is a group of wiki editors as well in case documentation is your thing: ardu-wiki-editors@googlegroups.com

>>Developer discussions occur on drones-discuss@google-groups.com

### License ###
>>[Overview of license](http://dev.ardupilot.com/wiki/license-gplv3)
# ArduPilot Project for F4BY

>>[Full Text](https://github.com/diydrones/ardupilot/blob/master/COPYING.txt)
## Building using make

 1. Before you build the project for the first time, you'll need to run `make
    configure` from a  sketch directory (i.e. ArduPlane, ArduCopter, etc...).
    This will create a `config.mk` file at the top level of the repository. You
    can set some defaults in `config.mk`

 2. In the sketch directory, type `make` to build for APM2. Alternatively,
    `make apm1` will build for the APM1 and `make px4` will build for the PX4.
    The binaries will generated in `/tmp/<i>sketchname</i>.build`.

 3. Type `make upload` to upload. You may need to set the correct default
    serial port in your `config.mk`.


## Development using VirtualBox

ardupilot has a standardized Linux virtual machine (VM) setup script
that uses the free VirtualBox virtualization software.  You can use it
to create a standard, reproducible development environment in just a
few minutes in Linux, OS X, or Windows.

 1. [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
 for your Mac, Windows or Linux machine.

 2. [Install vagrant](http://docs.vagrantup.com/v2/installation/).

 4. In the `ardupilot` directory, run `vagrant up` from the command
 line.  This will create a new Ubuntu Linux VM.

 5. Run `vagrant ssh -c "ardupilot/Tools/scripts/install-prereqs-ubuntu.sh -y"`.
 This will install all the prerequisites for doing ardupilot development.

You can now run `vagrant ssh` to log in to the development
environment.  The `~/ardupilot` directory in the VM is actually the
`ardupilot` directory in your host operating system--changes in either
directory show up in the other.

Once you've followed the instructions above, here's how you would
build ArduCopter for PX4 in the development environment:

```
$ vagrant ssh
# cd ardupilot/ArduCopter
# make configure
```

Back at the terminal:

```
# make px4
# make px4-upload  # (optional)
```

# User Technical Support

ArduPilot users should use the DIYDrones.com forums for technical support.

# Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also
join the [development discussion on Google
Groups](https://groups.google.com/forum/?fromgroups#!forum/drones-discuss). Note
that the Google Groups mailing lists are NOT for user tech support,
and are moderated for new users to prevent off-topic discussion.


