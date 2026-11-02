import numpy as np
import pytest

import tools.point_preview_cleanup as cleanup


def block(indices, depths=None):
    indices=np.asarray(indices,dtype='<u4')
    depths=np.full(len(indices),1000,dtype='<u2') if depths is None else np.asarray(depths,dtype='<u2')
    colors=np.arange(len(indices)*3,dtype=np.uint8).reshape(-1,3)
    return indices.tobytes()+depths.tobytes()+colors.tobytes()


def test_disabled_bypasses_all_neighbour_work_and_allocation(monkeypatch):
    raw=block([1,2,3])
    monkeypatch.setattr(cleanup,'selected_point_support',lambda *args:pytest.fail('Off ran neighbour work'))
    result,count=cleanup.clean_selected_points(raw,3,100,[1000,1000,0,0],{'point_cleanup_enabled':False})
    assert result is raw and count==3
    assert cleanup.clean_selected_points(raw,3,100,[1000,1000,0,0],{})[0] is raw


def test_selected_spatial_neighbours_remove_outlier_and_preserve_exact_samples():
    raw=block([0,1,2,100,101,102,200,201,202,5050]);original=bytes(raw)
    settings={'point_cleanup_enabled':True,'point_cleanup_support':3,'point_cleanup_radius_mm':2}
    result,count=cleanup.clean_selected_points(raw,10,100,[1000,1000,0,0],settings)
    assert count==9 and raw==original
    np.testing.assert_array_equal(np.frombuffer(result,'<u4',count=9),[0,1,2,100,101,102,200,201,202])
    assert result[9*4:9*6]==raw[10*4:10*4+9*2]
    assert result[9*6:]==raw[10*6:10*6+9*3]
    assert cleanup.clean_selected_points(raw,10,100,[1000,1000,0,0],{**settings,'point_cleanup_radius_mm':100})[1]==10
    assert cleanup.clean_selected_points(raw,10,100,[1000,1000,0,0],{**settings,'point_cleanup_support':16})[1]==0


def test_support_counts_only_points_selected_for_display():
    settings={'point_cleanup_enabled':True,'point_cleanup_support':1,'point_cleanup_radius_mm':2}
    # Intermediate pixels could support these points in a dense source image,
    # but they are not selected. They must not count as displayed neighbours.
    assert cleanup.clean_selected_points(block([0,5]),2,100,[1000,1000,0,0],settings)[1]==0
    assert cleanup.clean_selected_points(block([0,1,2,3,4,5]),6,100,[1000,1000,0,0],settings)[1]==6


@pytest.mark.parametrize('value',[0,1.5,17,True,float('nan')])
def test_support_validation(value):
    with pytest.raises(ValueError):
        cleanup.clean_selected_points(block([0]),1,100,[1000,1000,0,0],{'point_cleanup_enabled':True,'point_cleanup_support':value})


def test_native_matches_reference(monkeypatch):
    from tools.native_sparse_kernels import LIBRARY
    if LIBRARY is None:pytest.skip('Optional native kernels unavailable')
    rng=np.random.default_rng(25);indices=np.sort(rng.choice(4000,200,replace=False)).astype('<u4')
    depths=rng.integers(900,930,len(indices),dtype='<u2');raw=block(indices,depths)
    native=cleanup.selected_point_support
    for support in (1,3,16):
        settings={'point_cleanup_enabled':True,'point_cleanup_support':support,'point_cleanup_radius_mm':18}
        monkeypatch.setattr(cleanup,'selected_point_support',native)
        actual=cleanup.clean_selected_points(raw,len(indices),100,[1000,1000,50,20],settings)
        monkeypatch.setattr(cleanup,'selected_point_support',lambda *args:None)
        assert actual==cleanup.clean_selected_points(raw,len(indices),100,[1000,1000,50,20],settings)
