:- module(surfaces,
    [
      object_current_surface/2,
      object_goal_surface/2,
      objects_on_surface/2,
      assert_object_on/2,
      all_srdl_objects/1,
      shelf_floors/1,
      shelf_floor_at_height/2,
      table_surface/1,
      all_objects_in_whole_shelf/1,
      surface_pose_in_map/2,
      test_surfaces/0
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    object_current_surface(?,?),
    object_goal_surface(r,?),
    objects_on_surface(?,r),
    assert_object_on(r,r),
	all_srdl_objects(?),
    shelf_floors(?),
    shelf_floor_at_height(r,?),
    table_surface(?),
    all_objects_in_whole_shelf(?),
    surface_pose_in_map(r,?),
    test_surfaces.

object_current_surface(Instance, Surface) :-
    rdf_has(Instance, hsr_objects:'supportedBy', Surface).

%% If there's another object in the shelf with the same class, give same shelf
object_goal_surface(Instance, Surface) :-
    all_objects_in_whole_shelf(ObjectsInShelf),
    member(ObjectInShelf, ObjectsInShelf),
    findall(SingleClass, (
        rdfs_instance_of(ObjectInShelf, SingleClass)
        ), [Class|_]),
    rdfs_instance_of(Instance, Class),
    object_current_surface(ObjectInShelf, Surface), !.

%% If there is no corresponding class, take some shelf in the middle
object_goal_surface(_, Surface) :-
    (shelf_floor_at_height(0.9, Surface);
    shelf_floor_at_height(0.6, Surface)),
    objects_on_surface([], Surface), !.

%% If middle shelves also occupied, take rest (lowest level first). WARNING: HSR may not be able to reach upper levels
object_goal_surface(_, Surface) :-
    shelf_floors(ShelfFloors),
    member(Surface,ShelfFloors),
    objects_on_surface([], Surface).

objects_on_surface(Instances, Surface) :-
    findall(Instance,
        object_current_surface(Instance, Surface),
        Instances).

assert_object_on(Instance, Surface) :-
    all_srdl_objects(Surfaces),
    member(Surface,Surfaces),
    rdf_retractall(Instance, hsr_objects:'supportedBy', _),
    rdf_assert(Instance, hsr_objects:'supportedBy', Surface).

all_srdl_objects(Surfaces) :-
    findall(Surface,
        rdfs_individual_of(Surface, srdl2_comp:'UrdfLink'),
        Surfaces).

shelf_floors(ShelfFloors) :-
    findall(ShelfFloor, (
        all_srdl_objects(Surfaces),
        member(ShelfFloor, Surfaces),
        rdf_has_prolog(ShelfFloor, srdl2_comp:'urdfName', UrdfName),
        atom_concat('shelf_floor',_,UrdfName)
        ), ShelfFloors).

shelf_floor_at_height(Height, TargetShelf) :-
    findall(ShelfFloor, (
        shelf_floors(AllFloors),
        member(ShelfFloor, AllFloors),
        srdl_matrix(ShelfFloor, M),
        matrix(M, [_, _, Z], _),
        Z < Height
    ), ShelfFloors),
    reverse(ShelfFloors, [TargetShelf|_]).

table_surface(Table) :-
    all_srdl_objects(Surfaces),
    member(Table, Surfaces),
    rdf_equal(Table, robocup:'kitchen_description_table_surface_center').

all_objects_in_whole_shelf(Instance) :-
    findall(Instance, (
        shelf_floors(Shelves),
        member(Shelf, Shelves),
        objects_on_surface(ObjPerShelf, Shelf),
        member(Instance, ObjPerShelf)
        ), Instance).

surface_pose_in_map(Surface, [Translation, Rotation]) :-
    rdf_has(Surface, srdl2_comp:'urdfName', literal(UrdfName)),
    string_concat('environment/', UrdfName, Frame),
    tf_lookup_transform(map, Frame, PoseTerm),
    owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose),
    transform_data(Pose,(Translation, Rotation)).

test_surfaces :-
    owl_instance_from_class(hsr_objects:'Other', Instance),
    rdf_equal(Shelf, robocup:'kitchen_description_shelf_floor_1_piece'),
    assert_object_on(Instance, Shelf),
    rdfs_individual_of(Shelf, srdl2_comp:'UrdfLink'),
    objects_on_surface(Objects, Shelf),
    rdf_has(Instance, hsr_objects:'supportedBy', _),
    all_objects_in_whole_shelf(AllObjects),
    shelf_floors(AllFloors),
    member(Shelf, AllFloors),
    member(Instance,Objects),
    member(Instance,AllObjects),
    owl_instance_from_class(hsr_objects:'Other', OtherInstance),
    object_goal_surface(OtherInstance, OtherSurface),
    rdf_equal(Shelf, OtherSurface),
    srdl_matrix(Shelf, TestMatrix).

