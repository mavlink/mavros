^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_extras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2014-08-25)
------------------
* plugins: Change UAS FCU link name.
  Reduce smart pointer count, that hold fcu link object.
* Plugins: finish moving plugins
* Closes `#122 <https://github.com/vooon/mavros/issues/122>`_, closes `#123 <https://github.com/vooon/mavros/issues/123>`_; plugins: move mocap & vision plugins to extras, change vision plugins name
* launch: Add example launch for `#103 <https://github.com/vooon/mavros/issues/103>`_.
* extras: image_pub: Update plugin API.
* extras: px4flow: Update plugin API.
* plugins: disable most of plugins
* extras: init ctor
* extras: Fix package URLs
* test: temporary travis hack (manually download latest mavlink deb)
* Update readme
* Contributors: Nuno Marques, Vladimir Ermakov

0.7.0 (2014-08-12)
------------------
* move exras to subdirectory, `#101 <https://github.com/vooon/mavros/issues/101>`_
* Contributors: Vladimir Ermakov, M.H.Kabir
