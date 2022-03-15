^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sam_stonefish_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#24 <https://github.com/smarc-project/smarc_stonefish_sims/issues/24>`_ from luxiya01/kristineberg
  Kristineberg world
* Merge pull request `#23 <https://github.com/smarc-project/smarc_stonefish_sims/issues/23>`_ from KKalem/noetic-devel
  Added tree-watcher to the mission tmux pane
* Set default robot position and world config file to kristineberg
* Added tree-watcher to the mission tmux pane
* Merge pull request `#19 <https://github.com/smarc-project/smarc_stonefish_sims/issues/19>`_ from svbhat/noetic-devel
  Updated bringup script to separate out actions from BT, launch BT last.
* Merge pull request `#21 <https://github.com/smarc-project/smarc_stonefish_sims/issues/21>`_ from smarc-project/correct_thrusters
  Corrected error with RPMs not being set correctly
* Corrected everything to be parameters in defs
* Forgot to add parameter to other two vehicle defs
* Corrected error with RPMs not being set correctly
* Updated bringup script to separate out actions from BT, launch BT last.
* Update package.xml
* Merge pull request `#17 <https://github.com/smarc-project/smarc_stonefish_sims/issues/17>`_ from smarc-project/sim_dr
  Fixed UTM zone config issue
* Fixed UTM zone config issue
* Merge pull request `#16 <https://github.com/smarc-project/smarc_stonefish_sims/issues/16>`_ from smarc-project/sim_dr
  Corrected DR lat lon interface when simulating DR
* Corrected DR lat lon interface when simulating DR
* Merge pull request `#12 <https://github.com/smarc-project/smarc_stonefish_sims/issues/12>`_ from luxiya01/algae-sim-fixes
  Move algae farm position
* Correct algae farm UTM zone and band
* Merge pull request `#14 <https://github.com/smarc-project/smarc_stonefish_sims/issues/14>`_ from smarc-project/sim_dr
  Added compressed image topics to sim
* Added compressed image topics to sim
* Added compressed image topics to sim
* Merge pull request `#13 <https://github.com/smarc-project/smarc_stonefish_sims/issues/13>`_ from smarc-project/sim_dr
  Updated lcg actuation for both sam models
* Updated lcg actuation for both sam models
* Merge pull request `#11 <https://github.com/smarc-project/smarc_stonefish_sims/issues/11>`_ from smarc-project/sim_dr
  Added toggle DVL service
* Add small algae world to sam scenarios
* Added toggle DVL service
* Merge pull request `#10 <https://github.com/smarc-project/smarc_stonefish_sims/issues/10>`_ from smarc-project/sim_dr
  Slowed down acutators
* Slowed down LCG controller actuation
* Slowed down VBS controller actuation
* Move algae farm to ~correct coordinates
* Merge pull request `#9 <https://github.com/smarc-project/smarc_stonefish_sims/issues/9>`_ from smarc-project/sim_dr
  Modify simulation to work with DR
* Updated package versions
* Using same IMU parameters for both SAM models
* Using DR for positioning in bringup script
* Merged upstream
* Got IMUs to work with DR
* Got relative accs and vels working, global IMU orientation not working, needs ENU conversion
* Enabled running DR TF at the same time as sim TF
* Merge pull request `#8 <https://github.com/smarc-project/smarc_stonefish_sims/issues/8>`_ from smarc-project/nilsbore-patch-1
  Add dep on sam_basic_controllers
* Update package.xml
* Update package.xml
* Merge pull request `#6 <https://github.com/smarc-project/smarc_stonefish_sims/issues/6>`_ from svbhat/noetic-devel
  added controller launch files to bringup script. Commented out Nils' actions.
* Updated DR topic to be consistent with real SAM. Added dependency to tf_convenience_topics.
* added tf_convenience topics to robot_bridge.launch.Removed from bringup.sh.
* Added tf_convenience_topics to sam_stonefish_sim bringup.sh
* Merge pull request `#7 <https://github.com/smarc-project/smarc_stonefish_sims/issues/7>`_ from luxiya01/sss-sensor-interface
  Implement sensor interface for sidescan simulation
* Implement sensor interface for sidescan simulation
* Merge pull request `#3 <https://github.com/smarc-project/smarc_stonefish_sims/issues/3>`_ from luxiya01/add-sss
  Add side-scan sonar to SAM simulation
* Add conversion from stonefish sss to smarc_msgs::sidescan format
* Modify number of sss bins in stonefish simulation
* Rename simulation sss topic
  according to specification at https://github.com/smarc-project/smarc_msgs#id15
* Add SSS to SAM simulation
* Merge branch 'noetic-devel' of github.com:smarc-project/smarc_stonefish_sims into noetic-devel
* added controller launch files to bringup script. Removed Nils' actions.
* Update README.md
* Merge pull request `#2 <https://github.com/smarc-project/smarc_stonefish_sims/issues/2>`_ from smarc-project/merge_repos
  Add sam_stonefish_sim and lolo_stonefish_sim as subtrees
* Updated bringup scripts
* Updated package versions
* Removed duplicate CI configs
* Separated robot and world configs
* Moved common stuff to smarc_stonefish_worlds
* Moved worlds to live in smarc_stonefish_worlds
* Add 'sam_stonefish_sim/' from commit 'baa60338be0bcaef3c2feff6b977dc973582f3a8'
  git-subtree-dir: sam_stonefish_sim
  git-subtree-mainline: da0dfea00ef8f0bb30761dfc0507b40c69257055
  git-subtree-split: baa60338be0bcaef3c2feff6b977dc973582f3a8
* Contributors: Jollerprutt, Li Ling, Nils Bore, Ozer Ozkahraman, svbhat
