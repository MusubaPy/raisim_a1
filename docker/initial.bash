cd


mkdir -p ~/Zashchitin/workspace/raisim_workspace
mkdir -p ~/Zashchitin/workspace/raisim_build

cd Zashchitin/workspace/raisim_workspace

git clone https://github.com/raisimTech/raisimLib.git &&
git clone https://github.com/leggedrobotics/ogre.git &&
git clone https://github.com/raisimTech/raisimOgre.git &&
git clone https://github.com/pybind/pybind11.git

cd

cp -r Zashchitin/docker/req/rsg_a1 Zashchitin/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/

cp -r Zashchitin/docker/req/visualizer Zashchitin/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/

cp Zashchitin/docker/req/CMakeLists.txt Zashchitin/workspace/raisim_workspace/raisimLib/raisimGymTorch/

cp Zashchitin/docker/req/OgreGLXGLSupport.cpp Zashchitin/workspace/raisim_workspace/ogre/RenderSystems/GLSupport/src/GLX

cp Zashchitin/docker/req/RaisimGymEnv.hpp Zashchitin/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/

cp Zashchitin/docker/req/Reward.hpp Zashchitin/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/

cp -r .raisim workspace/

cp Zashchitin/docker/req/build.bash Zashchitin/workspace/raisim_workspace/
cp Zashchitin/docker/req/rebuild.bash Zashchitin/workspace/raisim_workspace/

cd

