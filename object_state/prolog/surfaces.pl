:- module(surfaces,
    [
      object_current_surface/2,
      object_goal_surface/2,
      objects_on_surface/2,
      assert_object_on/2,
      all_srdl_objects/1,
      shelf_floors/1,
      all_objects_in_whole_shelf/1,
      surface_size/2,
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
    all_objects_in_whole_shelf(?),
    surface_size(r,?),
    test_surfaces.

object_current_surface(Instance, Surface) :-
    rdf_has(Instance, hsr_objects:'supportedBy', Surface).

%% If there's another object in the shelf with the same class, give same shelf
object_goal_surface(Instance, Surface) :-
    rdfs_instance_of(Instance, Class),
    all_objects_in_whole_shelf(ObjectsInShelf),
    member(ObjectInShelf, ObjectsInShelf),
    rdfs_instance_of(ObjectInShelf, Class),
    object_current_surface(ObjectInShelf, Surface).

object_goal_surface(_, _) :-
    false.

objects_on_surface(Instances, Surface) :-
    findall(Instances,
        object_current_surface(Instances, Surface),
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

all_objects_in_whole_shelf(Instance) :-
    findall(Instance, (
        shelf_floors(Shelves),
        member(Shelf, Shelves),
        objects_on_surface(ObjPerShelf, Shelf),
        member(Instance, ObjPerShelf)
        ), Instance).

surface_size(TFFrame, Size) :-
    rdf_has(Instance, srdl2_comp:'urdfName', literal(TFFrame)),
    rdf_has(Instance, srdl2_comp:'box_size', Size), !.

surface_size(Instance, Size) :-
    rdf_has(Instance, srdl2_comp:'box_size', Size).

test_surfaces :-
    owl_instance_from_class(hsr_objects:'Other', Instance),
    Shelf = 'http://knowrob.org/kb/robocup.owl#kitchen_description_shelf_floor_1_piece',
    %rdf_equal(Shelf, robocup:'kitchen_description_shelf_floor_1_piece')
    assert_object_on(Instance, Shelf),
    rdfs_individual_of(Shelf, srdl2_comp:'UrdfLink'),
    objects_on_surface(Objects, Shelf),
    rdf_has(Instance, hsr_objects:'supportedBy', _),
    all_objects_in_whole_shelf(AllObjects),
    %shelf_floors(AllFloors),
    member(Instance,Objects),
    member(Instance,AllObjects),
    owl_instance_from_class(hsr_objects:'Other', OtherInstance),
    object_goal_surface(OtherInstance, OtherSurface).

