# ArduPilot Project for F4BY

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

<script src="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=RMC4D7C3Y64GJ"
    data-button="donate"
    data-name="My product"
    data-amount="1.00"
></script>
