# Contributing

1. Fork the repo. [<img src="https://upload.wikimedia.org/wikipedia/commons/3/38/GitHub_Fork_Button.png" height="30"/>](https://github.com/mavlink/mavros/fork)
2. Clone the repo into your workspace:

  ```bash
  git clone https://github.com/mavlink/mavros.git
  ```

3. Create a remote connection to your repo:

  ```bash
  git remote add origin git@github.com:<YourGitUser>/mavros.git
  ```

4. Create a feature/dev branch:

  ```bash
  git checkout -b <feature_branch>
  ```

5. Make your changes.
6. Commit the changes. The `-a` option automatically adds and removes files for you.

  ```bash
  git commit -a -m "<message>"
  ```

7. Check your code style:

  ```bash
  uncrustify -c ${ROS_WORKSPACE}/src/mavros/mavros/tools/uncrustify-cpp.cfg --replace --no-backup <path/to/file.ext>
  ```

8. Fix small code style errors and typos.
9. Commit with a description like "uncrustify" or "code style fix". Please avoid changes in program logic (separate commits are better than a mix of style and bug fixes).
10. Run tests:

- with `catkin_make`, issue `catkin_make tests` and then `catkin_make run_tests`;
- with `catkin tools`, issue `catkin run_tests`;

11. If everything goes as planned, push the changes and issue a pull request.

```bash
git push -u origin <feature_branch>
```

## cog.py generators

In many places we need to copy some data from MAVLink, and in many places we have regular patterns of code (e.g. copied message fields).
To avoid manual copy-paste work (and errors!) we use the the [cog.py][cog] code generator/preprocessor.

Cog generates C++ code from code blocks written in Python that you add into your C++ code file as specially formatted comments. Since you are now using Python, you can import and make use of the [pymavlink][pml] module, which already comes with MAVlink message definitions that you can reuse.
An example you may look at is the `utils::to_string()` implementation for some enums in [lib/enum_to_string.cpp][ets].

Install cog and pymavlink:

```bash
pip install --user cogapp pymavlink
```

Add your generator code to your file as comments enclosed in the `[[[cog:]]]` and `[[[end]]]` tags. Then invoke cog so that it updates your file:

```bash
cog.py -cr your_file.h/cpp
```

In addition, this script will re-generate all files with cog code:

```bash
./mavros/tools/cogall.sh
```

[cog]: https://nedbatchelder.com/code/cog/
[ets]: https://github.com/mavlink/mavros/blob/master/mavros/src/lib/enum_to_string.cpp
[pml]: https://mavlink.io/en/mavgen_python/
