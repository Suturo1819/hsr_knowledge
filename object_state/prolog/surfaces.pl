:- module(surfaces,
    [
      object_current_surface/2,
      object_goal_pose/2,
      object_goal_pose/3,
      object_goal_surface/2,
      object_goal_surface/3,
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
    object_goal_pose(r,?),
    object_goal_pose(r,?,?),
    object_goal_surface(r,?),
    object_goal_surface(r,?,?),
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

object_goal_pose(Instance, [Translation, Rotation]) :-
    object_goal_pose(Instance, [Translation, Rotation], _).

object_goal_pose(Instance, [Translation, Rotation], Context) :-
    object_goal_surface(Instance, Surface, Context),
    surface_pose_in_map(Surface, [[X,Y,Z], Rotation]),
    member(XOffset, [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, 0.35, 0.35]),
    NewX is X + XOffset,    
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z].

object_goal_pose(_Instance, [Translation, Rotation], Context) :-
    shelf_floor_at_height(0.2, Surface),
    surface_pose_in_map(Surface, [[X,Y,Z], Rotation]),
    member(XOffset, [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, 0.35, 0.35]),
    NewX is X + XOffset,    
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z].

object_goal_surface(Instance, Surface) :-
    object_goal_surface(Instance, Surface, _).

% Sort by size if object class is OTHER
object_goal_surface(Instance, Surface, Context) :-
    rdfs_type_of(Instance, hsr_objects:'Other'),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdf_has(Instance, hsr_objects:'size', Size),
    rdf_has(ShelfObj, hsr_objects:'size', Size),
    object_current_surface(ShelfObj, Surface),
    surface_pose_in_map(Surface, [[_,_,Z],_]),
    _SurfaceZInCentimeters is Z * 100,
    string_concat('I will put this to the other ', Size, Stringpart1),
    string_concat(Stringpart1, ' object on the shelf floor, which is ', Stringpart2),
    string_concat(Stringpart2, _SurfaceZInCentimeters, Stringpart3),
    string_concat(Stringpart3, ' centimeters above the ground.', Context).

% Sort by color, if object class is OTHER
object_goal_surface(Instance, Surface, Context) :-
    rdfs_type_of(Instance, hsr_objects:'Other'),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdf_has(Instance, hsr_objects:'colour', Color),
    rdf_has(ShelfObj, hsr_objects:'colour', Color),
    object_current_surface(ShelfObj, Surface),
    surface_pose_in_map(Surface, [[_,_,Z],_]),
    SurfaceZInCentimeters is Z * 100,
    string_concat('I will put this to the other ', Color, Stringpart1),
    string_concat(Stringpart1, ' object.', Context).

%    string_concat(Stringpart1, ' object on the shelf floor, which is ', Stringpart2),
%    string_concat(Stringpart2, SurfaceZInCentimeters, Stringpart3),
%    string_concat(Stringpart3, ' centimeters above the ground.', Context).

%% Same obj class
object_goal_surface(Instance, Surface, Context) :-
    rdfs_type_of(Instance, Class),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_instance_of(ShelfObj, Class),
    object_current_surface(ShelfObj, Surface),
    rdf_split_url(_, CName, Class),
    string_concat('I will put this to the other ', CName, Context).

%% Same direct superclass
object_goal_surface(Instance, Surface, Context) :-
    rdfs_type_of(Instance, Class),
    owl_direct_subclass_of(Class, Super),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_instance_of(ShelfObj, Super),
    object_current_surface(ShelfObj, Surface),
    rdf_split_url(_, CName, Super),
    string_concat('I will put this to the other ', CName, Context).

%% Same superclass 2 levels up
object_goal_surface(Instance, Surface, Context) :-
    rdfs_type_of(Instance, Class),
    owl_direct_subclass_of(Class, Super),
    owl_direct_subclass_of(Super, Supersuper),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_instance_of(ShelfObj, Supersuper),
    object_current_surface(ShelfObj, Surface),
    rdf_split_url(_, CName, Supersuper),
    string_concat('I will put this to the other ', CName, Context).



%% If there's another object in the shelf with the same class, give same shelf
%object_goal_surface(Instance, Surface, Context) :-
%    all_objects_in_whole_shelf(ObjectsInShelf),
%    member(ObjectInShelf, ObjectsInShelf),
%    findall(SingleClass, (
%        rdfs_instance_of(ObjectInShelf, SingleClass)
%        ), [Class|_]),
%    rdfs_instance_of(Instance, Class),
%    object_current_surface(ObjectInShelf, Surface).

%% If there is no corresponding class, take some shelf in the middle
object_goal_surface(_, Surface, Context) :-
    (shelf_floor_at_height(0.9, Surface);
    shelf_floor_at_height(0.6, Surface)),
    objects_on_surface([], Surface),
    Context = 'I will create a new group for this'.

%% If middle shelves also occupied, take rest (lowest level first). WARNING: HSR may not be able to reach upper levels
object_goal_surface(_, Surface, Context) :-
    shelf_floors(ShelfFloors),
    member(Surface,ShelfFloors),
    objects_on_surface([], Surface),
    Context = 'I will create a new group for this'.


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
    srdl_matrix(Shelf, _).

