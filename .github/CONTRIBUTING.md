Contributing
============


1. Fork the repo:
   ![fork](http://s24.postimg.org/pfvt9sdv9/Fork_mavros.png)
2. Clone the repo (`git clone https://github.com/mavlink/mavros.git`);
3. Create a remote connection to your repo (`git remote add <remote_repo> git@github.com:<YourGitUser>/mavros.git`);
4. Create a feature/dev branch (`git checkout -b <feature_branch>`);
5. Add the changes;
6. Apply the changes by committing (`git commit -m "<message>"` or `git commit -a` and then write message; if adding new files: `git add <path/to/file.ext>`);
7. Check code style `uncrustify -c ${ROS_WORKSPACE}/src/mavros/mavros/tools/uncrustify-cpp.cfg --replace --no-backup <path/to/file.ext>`;
8. Fix small code style errors and typos;
9. Commit with description like "uncrustify" or "code style fix". Please avoid changes in program logic (separate commit are better than mix of style and bug fix);
10. Run tests:
   - with `catkin_make`, issue `catkin_make tests` and then `catkin_make run_tests`;
   - with `catkin tools`, issue `catkin run_tests`;
11. If everything goes as planned, push the changes (`git push -u <remote_repo> <feature_branch>`) and issue a pull request.


cog.py generators
-----------------

In many places we need to copy some data from MAVLink, and in many places we have regular patterns of code (e.g. copy message fields).
To avoid manual copy-paste work (and errors!) we use [cog.py][cog] generator/preprocessor.
Generator program written in comment blocks on Python (that allow import pymavlink), output will be inserted between markers.
As an example you may look at `utils::to_string()` implementation for some enums: [lib/enum_to_string.cpp][ets].

To install it :

    pip install --user cogapp pymavlink

Then fill the behaviour you when between the `[[[cog:]]]` `[[[end]]]` balise
and invoke cog like this:

    cog.py -cr your_file.h/cpp

Your file will be updated by cog.

    ./mavros/tools/cogall.sh

This script will regenerate all files with generators.


[cog]: https://nedbatchelder.com/code/cog/
[ets]: https://github.com/mavlink/mavros/blob/master/mavros/src/lib/enum_to_string.cpp
