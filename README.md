# PUT / AI Robotics 1 laboratories

This framework will gently introduce you to Docker in the context of Robotics classes. You're highly encouraged to **_explore all the scripts_** in this repo so you can add Docker to your toolset (**_they are annotated!_**). The suggested workflow will be presented next.

## Outline

First of all, you'll need to build an image for each laboratory class. You can do so by using the **_robbuild_** script. You are recommended to work on the assignments inside the **_shared_** directory. The **_robcreate_** will automatically mount it to the root of a container making it a _bridge_ between the host OS and the container. The **_robget_** will download the needed **_materials_** and the **_robshare_** will help you move _catkin packages_ and _symlink_ them back. _An example is below..._ 👇

```
# TODO
```

## More on scripts

In Linux, a script needs special permission to be executed. Grant it by typing `chmod +x <path-to-script>`.

To run a script type `<path-to-script> <argument1> <argument2> ... ` and until it isn't an alias, **_remember about ./ if in the script directory and an extension (.sh)_**.

One of the argument notations:

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
        <td>-a/-aq/--quiet</td>
        <td>dash is usually a prefix for non-positional arguments; you may try to combine them; double dash is used for long forms</td>
    </tr>
    <tr>
        <td>update|upgrade</td>
        <td>mutually exclusive options are separated by <code>|</code> - pick one</td>
    </tr>
</table>

Available scripts: **robbuild**, **robcreate**, **robget**, **robshare** (run this one from a container; you can use an identical alias).

_Go see help for each of them_ (`./<script>.sh` without any arguments from the script directory).

## WSL configuration

TODO
