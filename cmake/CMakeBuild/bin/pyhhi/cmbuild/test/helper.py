import os.path
import pyhhi.build.common.util as util


def get_test_data_top_dir():
    """Returns the default test data directory based on the script directory
    avoiding hardcoded paths while switching between BoostBuild and CMakeBuild."""

    script_dir_up = os.path.normpath(os.path.join(util.get_script_dir(), '..'))
    repo_name = os.path.basename(script_dir_up)
    # repo_name: either BoostBuild or CMakeBuild
    top_dir = util.get_top_dir()
    test_data_dir = os.path.join(top_dir, 'TestData', repo_name)
    assert os.path.exists(test_data_dir)
    return test_data_dir
