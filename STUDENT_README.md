# ECSE211 Project

_Read this entire document before doing anything._

This is the repository that contains the required files for the beta demo and final project.
For beta demo and competition requirements, see detailed instructions on MyCourses.

## Submission instructions

From now on, your hardware, software, and testing documents must refer to commits made in this repo,
to allow us to easily link your documents with specific versions of your design artifacts. You should make **GitHub releases** for
every major version of your codebase (eg, every week, or before important milestones like the beta demo).

For detailed instructions on how to do this, see [here](https://mcgill-dpm.github.io/website/GithubReleases).

Your beta demo release must be named `beta` and your final competition release must be named `final`. Both are _all lowercase_.
Your other releases should have names in the form `v3.1`, to indicate a specific version.

GitHub automatically creates zip files for each release you make,
which makes it easy for you submit them when requested. Note that for your
submission to be complete, all your work must be committed, including
generated javadoc.

## Updates and corrections

As with the labs, we will post any updates, corrections, and
clarifications to the starter code or these instructions on
[this page](https://mcgill-dpm.github.io/website/lab-notes).
Please check it regularly. We will notify you of major updates.

## Design Process

Remember to continue following the **design process** emphasized in this course. Document all your work.

## Prerequisites

Make sure you have set up your system as described in the previous labs
and the Getting Started Guide.

- [ ] Java 11 or higher installed and accessible from the terminal
(command line). Run `java`, `javac`, and `jar` to verify this.

- [ ] LeoCAD and Webots. You must be able to build and run the simulation
from your own machine, without relying on another team member.

- [ ] Eclipse. Make sure you use the Eclipse preferences file,
[CheckStyle](https://mcgill-dpm.github.io/website/EclipseCheckstyleSpotbugs),
and set the `WEBOTS_HOME` [classpath variable](https://mcgill-dpm.github.io/website/EclipseClasspathVariables)
if you have not already done so in Lab 3.

- [ ] GNU Awk (optional). This is required for generating HTML test reports 
with `make test`. More information is available [here](https://mcgill-dpm.github.io/website/JUnit).


## Implementation Details

**Important:** Be sure to set your team name and/or number in all of the following places:

* [`README.md`](README.md): in the title.
* [`.project`](.project): on line 3, replace the `xx` in `dpm-project-txx`
with your team number. Add a zero before your team number if it is a
single digit (eg, `07`).
* Rename `TXXController` according to your team name (eg, `T04Controller`, `T19Controller`). Rename both the file and the folder.
You can use the refactor menu in Eclipse to do this easily.
* `Resources.java`: set the `TEAM_NUMBER` constant to your team number.
* The `example_data_fill` files in `server/`, to avoid retyping your team
number every time you run the server.
* Your hardware proto files must be prefixed with `TXX`, where `XX` is your team number.
Add a zero before your team number if it is a single digit (eg, `07`).
* Your robot name must be `TXXRobot`.
___

The general project project structure is the same as Lab 5, with the addition of the (virtual) Wi-Fi server and its parameters.
For instructions on how to use them, see [here](https://mcgill-dpm.github.io/website/Wifi).

Refer to the Lab 5 readme and [this link](https://mcgill-dpm.github.io/website/JUnit)
for instructions on using JUnit and generating Javadoc API documents.

Please note that we will decide what parameters the server will send to your robot.
As a result, your code must be able to handle **any** valid input as specified in the project description,
not just the example given here.
