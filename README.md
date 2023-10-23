# rosetta_launch
A guide to understanding launch files in ROS 1 and ROS 2

This repository contains two packages, named after adolescent genetic variants of turtles.
 * `raphael` is a ROS 1 package
 * `donatello` is a ROS 2 package

Donatello is more technically advanced, but is sometimes harder to understand.

## 01 - Launch a Single Node
Three things are required for the minimal example:
 * Name - Used for node name, regardless of executable name
 * Package Name - Name of the ROS Package
 * Type - Name of the executable in the ROS package

### ROS 1

[source](raphael/launch/01-single.launch)
```xml
<launch>
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" />
</launch>
```

### ROS 2

[source](donatello/launch/01-single.launch.py)
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(name='does_machines', package='donatello', executable='donatello_node'),
    ])
```
You can choose whether you want to use the `import X` or `from X import Y` syntax. This guide uses the latter for the `launch` and `launch_ros` imports.


There are actually several different "styles" for adding nodes to a launch description.
 1. You can construct the nodes directly in the list passed to the `LaunchDescription` constructor (as above)
 2. You can assign the nodes to a variable and then put them in the `LaunchDescription` constructor at the end.
    ```python
    don_node = Node(name='does_machines', package='donatello', executable='donatello_node')
    return LaunchDescription([don_node])
    ```
 3. You can construct the `LaunchDescription` first with no nodes, and then add them individually.
    ```python
    ld = LaunchDescription()
    ld.add_action(Node(name='does_machines', package='donatello', executable='donatello_node'))
    return ld
    ```


## 02 - Set a parameter directly
If we want to specify an exact value for a ROS parameter inside of the launch file, we need the parameter name and the parameter value.

### ROS 1
[source](raphael/launch/02-param.launch)
```xml
<launch>
    <param name="use_sim_time" value="true" />

    <node name="cool_but_rude" pkg="raphael" type="raphael_node" output="screen">
        <param name="pizza" value="pepperoni" />
        <param name="brothers" type="yaml" value="[leo, don, mike]"/>
        <rosparam param="coworkers">[leo, don, mike]</rosparam>
        <rosparam>
            weapon: sai
        </rosparam>
    </node>
</launch>
```

 * We can set a global parameter in ROS 1 (e.g. `use_sim_time`)
 * The other parameters are set within the node, and thus will be given the node's namespace, i.e. the full parameter will be `/cool_but_rude/pizza`.
 * For simple types (`str|int|double|bool`), you add the `param` xml element with the name and value (e.g. `pizza`). The type of the parameter is automatically inferred.
 * For more complex types, you can specify the type as `yaml` and the value will be interpreted as yaml, (e.g. `brothers`)
 * The contents of a `rosparam` xml element can also be interpreted as yaml. You can specify the name as an attribute (e.g. `coworkers`) or specify a whole dictionary of names and values (e.g. `weapon`)

### ROS 2
[source](donatello/launch/02-param.launch.py)
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[{'pizza': 'mushrooms',
                          'brothers': ['leo', 'mike', 'raph']}]),
    ])
```
 * You cannot set global parameters in ROS 2.
 * The parameters are specified using the `parameters` argument within the Node, which takes a Python list.
 * To specify the values directly, we put them in a dictionary within the Python list.
 * Since the parameters are within Python code, the type is determined by their Python type.


## 03 - Load Parameters from YAML file

To load parameters from a file, we need the full path to the yaml file.

### ROS 1
[source](raphael/launch/03-params.launch)
```xml
<launch>
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" output="screen">
        <rosparam command="load" file="$(find raphael)/config/params.yaml" />
    </node>
</launch>
```
 * In the `rosparam` element, we now use `command="load"` and specify the full path with the `file` attribute.
 * To automatically get the full path to a file within a ROS package, we can use the special `$(find package_name)` syntax.


### ROS 2
[source](donatello/launch/03-params.launch.py)
```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[PathJoinSubstitution(FindPackageShare('donatello'), 'config', 'params.yaml')]),
    ])
```

* In the `parameters` argument of the Node, we previously specified a dictionary to give values to the parameters individually. To load the parameters from a file, we must specify the full path to the file, which we can do in three ways.
  1. **FindPackageShare + PathJoinSubstitution** (as shown above) - Instead of a dictionary, we now specify the path as a combination of the share directory and the other directory components all joined together. (See more information on Substitutions in a section below)
  2. **String** - You can pass in a string representing the path. This is often combined with `ament_index_python.packages.get_package_share_directory` and `os.path.join`, i.e.
    ```python
    parameters=[
        os.path.join(get_package_share_directory('donatello'),
                     'config/params.yaml'
                     )
                 ]
    ```
  3. **Path converted to a String** - Working with `pathlib` is often easier than using `os.path.join`. You can get a `pathlib.Path` of the share folder using `ament_index_python.package.get_package_share_path` i.e.
    ```python
    parameters=[
        str(get_package_share_path('donatello') / 'config/params.yaml')
                 ]
    ```

* While the latter two ways of specifying the path are common, if you are combining with other substitutions, the first option is the easiest to work with.


## 04 - Load Parameters from a Command
Sometimes you will want to set parameters based on the results of running a command. This is very commonly seen when running `xacro` on your robot model and loading it in as a parameter.

### ROS 1
[source](raphael/launch/04-command-params.launch)
```xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro $(find urdf_tutorial)/urdf/01-myfirst.urdf" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
</launch>
```

### ROS 2
[source](donatello/launch/04-command-params.launch.py)
```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(FindPackageShare('urdf_tutorial'), 'urdf', '01-myfirst.urdf')]),
        value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}])
    ])
```

 * Note: In many cases, you can get away with not wrapping the `Command` in a `ParameterValue` object, but then the launch system will try to guess the value type, and if there happens to be a colon (`:`) in a string you're trying to load, [it will try to interpret it as YAML](https://github.com/ros2/launch_ros/issues/214)

## 05 - Set a Command Line Argument
There are two different steps for using command line arguments:
 * Declaring the argument and its default value
 * Using the value

### ROS 1
[source](raphael/launch/05-arg.launch)
```xml
<launch>
    <arg name="pizza_type" default="pepperoni" />
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" output="screen">
        <param name="pizza" value="$(arg pizza_type)" />
    </node>
</launch>
```

 * The argument is declared using the `<arg>` element.
 * The value can be using the dollar substitution syntax `$(arg pizza_type)`
 * When launching, you can see the value of the parameter printed to the terminal.
   ```
   PARAMETERS
    * /cool_but_rude/pizza: pepperoni
   ```
 * If we run `roslaunch raphael 05-arg.launch`, the parameter is set to pepperoni by default.
 * You can specify the value you want on the command line with `arg_name:=arg_value`, i.e.
   ```bash
   roslaunch raphael 05-arg.launch pizza_type:=meatball
   ```

### ROS 2
[source](donatello/launch/05-arg.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[{'pizza': LaunchConfiguration('pizza_type')}]),
    ])
```
 * The argument is declared using the `DeclareLaunchArgument` action, which must be included in the `LaunchDescription`
 * The value can be used with the `LaunchConfiguration` object.
 * If we run `ros2 launch donatello 05-arg.launch.py` the parameter is set to `mushrooms` by default.
 * You can specify the value you want on the command line the same way as ROS 1, i.e.
   ```bash
   ros2 launch donatello 05-arg.launch.py pizza_type:=meatball
   ```

## 06 - Using Substitutions Everywhere
In the previous example, we were able to dynamically change our launch file by using substitutions, i.e. dynamically replacing the value of a variable. We also used `FindPackageShare` to dynamically find the path to a file. There are actually a lot of different things you can substitute in.

| Name                          | ROS 1 command | ROS 2 command          | ROS2 Object         | Note |
|-------------------------------|---------------|------------------------|---------------------|------|
| Launch Argument               | arg           | var                    | LaunchConfiguration | [1]  |
| Anonymous ID Generation       | anon          | anon                   | AnonName            |      |
| Environment Variable          | env           | env                    | EnvironmentVariable |      |
| Optional Environment Variable | optenv        | env                    | EnvironmentVariable | [2]  |
| Command Execution             |               | command                | Command             | [3]  |
| Current Directory             | dirname       | dirname                | ThisLaunchFileDir   |      |
| ROS Package Location          | find          |                        |                     | [4]  |
| Current Launch File Path      |               | filename               | ThisLaunchFile      | [4]  |
| Executable Path               |               | find-exec              | FindExecutable      | [4]  |
| ROS Executable                |               | exec-in-pkg            | ExecutableInPackage | [4]  |
| ROS Package Share Path        |               | find-pkg-share         | FindPackageShare    | [4]  |
| Expression Evaluation         | eval          | eval                   | PythonExpression    | [5]  |

 * [1] Only one with inexplicable different command in ROS 1 and 2
 * [2] To use `optenv` in ROS 2, just add the default value to the command as the second argument.
 * [3] There is not a general way to load the results of an arbitrary command in ROS 1, but you can load the value into a ROS Parameter (See section 04 above)
 * [4] ROS 2 if much finickier about where files actually are. There is no general folder that covers everything, thus the `find` command doesn't make sense. Instead its split up into other commands.
 * [5] In ROS 1, `eval` cannot be used in YAML files.

### ROS 1
[source](raphael/launch/06-substitutions.launch)
```xml
<launch>
    <arg name="radius" default="1.5" />
    <arg name="pizza_type" default="pepperoni" />
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" output="screen">
        <param name="pizza" value="$(arg pizza_type)" />
        <param name="circumference" value="$(eval 2.* 3.1415 * arg('radius'))"/>
        <rosparam command="load" file="$(find raphael)/config/sub_params.yaml" subst_value="true"/>
    </node>
</launch>
```

[parameter file source](raphael/config/sub_params.yaml)
 * [Additional Documentation](http://wiki.ros.org/roslaunch/XML/#substitution_args)
 * Just like with args, we can still use the dollar substitution syntax in any string in the XML, e.g. `$(COMMAND ARG1 ARG2...)`
 * We can also put dollar substitutions in yaml files, as long as we set `subst_value="true"`
 * It may be relatively obvious, but the substitution replaces the dollar expression with the evaluated value, and then the yaml is processed as normal.
   * Example:`version: ROS $(env ROS_VERSION)` sets `version` to `ROS 1`
 * You can use launch arguments within `eval` substitutions, but with a slightly different syntax.

### ROS 2
There are multiple ways to do substitutions in ROS 2 Python launch files.

[source](donatello/launch/06a-substitutions.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import AnonName, Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch.substitutions import PythonExpression, ThisLaunchFile, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    object_parameters = {
        'pizza': LaunchConfiguration('pizza_type'),
        'anonymous_name': AnonName('leo'),
        'favorite_brother': EnvironmentVariable('BROTHER_NAME', default_value='mikey'),
        'filename': ThisLaunchFile(),
        'directory': ThisLaunchFileDir(),
        'list_exec': FindExecutable(name='ls'),
        'list_output': Command('ls'),
        'version': ['ROS ', EnvironmentVariable('ROS_VERSION')],
        'circumference': PythonExpression(['2.*3.1415*', LaunchConfiguration('radius')]),
    }

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[object_parameters]),
    ])
```

 * The most Python-y way to do substitutions is by using the `launch.substitutions` classes, as seen above.
 * When evaluating the substitutions, lists of strings/substitutions are **concatenated**. For example, look at `version` above.
   * The parameter value is a list containing one string and one substitution. The `EnvironmentVariable` is evaluated, resulting in the list now being `['ROS ', 2]`, which are then combined so the final value is `'ROS 2'`
 * The same principles apply to the substitution within a substitution in `circumference`.

[source](donatello/launch/06b-substitutions.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.frontend.parse_substitution import parse_substitution


def generate_launch_description():
    text_params = {
        'pizza': parse_substitution('$(var pizza_type)'),
        'anonymous_name': parse_substitution('$(anon leo)'),
        'favorite_brother': parse_substitution('$(env BROTHER_NAME mikey)'),
        'filename': parse_substitution('$(filename)'),
        'directory': parse_substitution('$(dirname)'),
        'list_exec': parse_substitution('$(find-exec ls)'),
        'list_output': parse_substitution('$(command ls)'),
        'version': parse_substitution('ROS $(env ROS_VERSION)'),
        'circumference': parse_substitution('$(eval 2.*3.1415*$(var radius))'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[text_params]),
    ])
```

 * You can also use the dollar substitution syntax as in ROS 1.
 * Using the `parse_substitution` method will result in lists and objects as in the previous example.

[source](donatello/launch/06c-substitutions.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    file_parameters = ParameterFile(
        param_file=PathJoinSubstitution(FindPackageShare('donatello'), 'config', 'sub_params.yaml'),
        allow_substs=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[file_parameters]),
    ])
```

[parameter file source](donatello/config/sub_params.yaml)
 * In ROS distributions [Galactic](https://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html#use-launch-substitutions-in-parameter-files) and newer, you can also use the substitutions in YAML files as well, as long as `allow_substs` is set to True.

## 07 - Include Another Launch

In more complex systems, it is often useful to have launch files that include other launch files, often including specific values for the launch arguments.

### ROS 1
[source](raphael/launch/07-inclusive.launch)
```xml
<launch>
    <include file="$(find raphael)/launch/05-arg.launch">
        <arg name="pizza_type" value="olives" />
    </include>
</launch>
```
### ROS 2
[source](donatello/launch/07-inclusive.launch.py)
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('donatello'), 'launch', '05-arg.launch.py'),
                                 launch_arguments={'pizza_type': 'peppers'}.items()),
    ])
```

 * The first argument to the `IncludeLaunchDescription` object in this case is a string representing the path to the launch file to include, which automatically gets converted to a [`launch.launch_description_source.PythonLaunchDescriptionSource`](https://github.com/ros2/launch/blob/b129eb65c9f03980c724b17200236290fa797816/launch/launch/launch_description_sources/python_launch_description_source.py) class, which is a subclass of [`launch.LaunchDescriptionSource`](https://github.com/ros2/launch/blob/b129eb65c9f03980c724b17200236290fa797816/launch/launch/launch_description_source.py#L30)
 * There are cases where you would want to construct the `PythonLaunchDescriptionSource` explicitly, or include a different type of `LaunchDescriptionSource`, but for now the string with the full path to the Python is the most straightforward option.

## 08 - Conditionally Include
In this example, we combine the substitution functionality and the ability to include another launch file from the last two examples to evaluate an argument to determine which launch file to include. We're going to include both cases, i.e. including if the argument is true and if the argument is false, but you can do just one.

### ROS 1
[source](raphael/launch/08-conditional.launch)
```xml
<launch>
    <arg name="use_number_one" default="true" />
    <include file="$(find raphael)/launch/01-single.launch"    if="$(arg use_number_one)" />
    <include file="$(find raphael)/launch/02-param.launch" unless="$(arg use_number_one)" />
</launch>
```

 * [Additional Documentation](http://wiki.ros.org/roslaunch/XML/#if_and_unless_attributes)
 * If we run `roslaunch raphael 08-conditional.launch`, it will include `01-single.launch` (and no parameters are loaded).
 * To get the same result, we can also explicitly set `use_number_one` to `true` or `1`.
 * If we run `roslaunch raphael 08-conditional.launch use_number_one:=false` it will include `02-param.launch` (with the parameters set in that launch file)
 * We could have also set `use_number_one:=0`, but any other values besides 0, 1, true, false will result in `Value error: X is not a 'bool' type`
 * `if/unless` can be used on individual nodes as well.

### ROS 2
[source](donatello/launch/08-conditional.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_number_one', default_value='True'),
        IncludeLaunchDescription(
            PathJoinSubstitution(FindPackageShare('donatello'), 'launch', '01-single.launch.py'),
            condition=IfCondition(LaunchConfiguration('use_number_one')),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(FindPackageShare('donatello'), 'launch', '02-param.launch.py'),
            condition=UnlessCondition(LaunchConfiguration('use_number_one')),
        ),
    ])
```
 * Same behaviors as in ROS 1:
   * `ros2 launch donatello 08-conditional.launch.py`
   * `ros2 launch donatello 08-conditional.launch.py use_number_one:=False`
 * Can raise `InvalidConditionExpressionError` if the argument is not a bool-ish type.
 * The `condition` parameter can be used on a wide array of Python launch objects, including individual nodes.


## 09 - Dynamic Filenames

One other way to dynamically change the contents of a launch file by evaluating substitutions is by using substitutions to determine the filenames.

### ROS 1
[source](raphael/launch/09-dynamic-filename.launch)
```xml
<launch>
    <arg name="config" default="params" />
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" output="screen">
        <rosparam command="load" file="$(find raphael)/config/$(arg config).yaml" />
    </node>
</launch>
```
 * Running `roslaunch raphael 09-dynamic-filename.launch` will load `params.yaml` by default.
 * Running `roslaunch raphael 09-dynamic-filename.launch config:=alt_params` will load `alt_params.yaml`
 * This method can also be used for changing the filename of an included launch file.

### ROS 2
[source](donatello/launch/09-dynamic-filename.launch.py)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamic_param_path = PathJoinSubstitution(
        FindPackageShare('donatello'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='params'),
        Node(package='donatello',
             executable='donatello_node',
             name='does_machines',
             parameters=[dynamic_param_path]),
    ])
```

 * Normally, (i.e. section 3 above) we pass a string of the parameters file into the list of parameters for the `Node`.
 * However, now we pass in a list (`dynamic_param_path`), which consists of a mix of strings and `launch.substitutions`, i.e. `LaunchConfiguration`
 * At runtime, the list will be evaluated and combined into one long string representing the path that will be loaded.

## 10 - Make A Node Required
There are certain scenarios where you want to stop an entire launch file when a particular node is not running anymore.
### ROS 1
[source](raphael/launch/10-required.launch)
```xml
<launch>
    <node name="cool_but_rude" pkg="raphael" type="raphael_node" />
    <node name="five_seconds" pkg="raphael" type="five_seconds" required="true" />
</launch>
```
 * The node `five_seconds` will terminate after five seconds.
 * Because it is marked as `required`, the other nodes in the launch file (`cool_but_rude`) will also be terminated after five seconds.

### ROS 2
[source](donatello/launch/10-required.launch.py)
```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    don_node = Node(name='does_machines', package='donatello', executable='donatello_node')
    five_node = Node(name='five_seconds', package='donatello', executable='five_seconds')
    handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=five_node,
            on_exit=[
                LogInfo(
                    msg='Five node exited; tearing down entire system.'),
                EmitEvent(
                    event=Shutdown())]))

    return LaunchDescription([don_node, five_node, handler])
```
 * Thanks to [The Ubuntu Blog](https://ubuntu.com/blog/ros2-launch-required-nodes) for their useful example.
 * Like so many things in ROS 2, the process for making a node required is more complicated, but also more flexible.


## 11 - Launch from Code
If you want to start a launch file from a script, you can sometimes use the `subprocess` Python library, but that can lead to problems with the paths. This is how you can launch a launch file in the ROS-y way.

### ROS 1
[source](raphael/scripts/manual_launch)
```python
#!/usr/bin/python3
from catkin.find_in_workspaces import find_in_workspaces
from roslaunch.parent import ROSLaunchParent
from roslaunch import rlutil


def main():
    uuid = rlutil.get_or_generate_uuid(None, False)
    launch_path = find_in_workspaces(project='raphael', path='launch/05-arg.launch', first_match_only=True)[0]
    launch_with_args = (launch_path, ['pizza_type:=extra_cheese'])

    p = ROSLaunchParent(uuid, [launch_with_args])
    p.start()
    p.spin()


if __name__ == '__main__':
    main()
```
 * This version starts `roscore`

### ROS 2
[source](donatello/donatello/manual_launch.py)
```python
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file


def main():
    path = get_share_file_path_from_package(package_name='donatello', file_name='05-arg.launch.py')
    launch_arguments = ['pizza_type:=extra_cheese']

    launch_a_launch_file(
        launch_file_path=path,
        # Note: launch_file_arguments is required!
        launch_file_arguments=launch_arguments,
    )


if __name__ == '__main__':
    main()
```

# Other Links
 * ROS 1
   * [ROS Wiki](http://wiki.ros.org/roslaunch/XML) with description of each XML element.
   * [Source Code](https://github.com/ros/ros_comm/tree/noetic-devel/tools/roslaunch/src/roslaunch)
   * [Architecture](http://wiki.ros.org/roslaunch/Architecture)
 * ROS 2
   * [Architecture](https://github.com/ros2/launch/blob/rolling/launch/doc/source/architecture.rst)
   * Source code for [launch](https://github.com/ros2/launch/tree/rolling/launch/launch) and [launch_ros](https://github.com/ros2/launch_ros/tree/rolling/launch_ros/launch_ros) which are different for...reasons.
