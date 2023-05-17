# PUT / AI Robotics 1 laboratories

This framework will gently introduce you to Docker in the context of Robotics classes. You're highly encouraged to **_explore all the scripts_** in this repo so you can add Docker to your toolset (**_they are annotated!_**). The suggested workflow will be presented next.

## Outline

First of all, you'll need to build an image for each laboratory class. To do so you'll need the _robbuild_ script. You are recommended to work on the assignments inside the _shared_ directory. The _robrun_ script automatically mounts it to the root of a container making it a bridge between the host OS and the container. Inside you'll find a directory for each laboratory containing a _solution_ directory. The _robget_ script will download the needed _materials_ next to it. The _robshare_ script will help you move catkin packages to _shared_ and symlink them back.

## More on scripts

Argument notation:

<table>
    <tr>
        <td>&lt;file&gt;</td>
        <td>angle brackets and their content should be replaced by a respective value</td>
    </tr>
    <tr>
        <td>[--shallow-submodules]</td>
        <td>everything inside the square brackets is optional</td>
    </tr>
    <tr>
        <td>-h/--help</td>
        <td>dash is usually a prefix for non-positional arguments; double dash is used for long forms</td>
    </tr>
    <tr>
        <td>{update | upgrade}</td>
        <td>curly brackets contain mutually exclusive options; pick one</td>
    </tr>
</table>

Available scripts: **robbuild**, **robrun**, **robget**, **robshare** (run this one from a container; you can use an identical alias)

To run a script type `./path-to-script argument1 argument2 ...`; until it isn't an alias, **_remember about ./ and an extension (.sh)_**

_Now go see help for each of the scripts_ (**_./script.sh -h_** from the script directory)

## How the build works
