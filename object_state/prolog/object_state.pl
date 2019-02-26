
:- module(object_state,
    [
      create_object/2,
      object_at/4,
      test_belief/0,
      spawn_on_table/0,
      clear/0,
      table_transform/0,
      object_at_table/1,
      object_of_type/2
    ]).

/*:- use_module(library('lists')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/objects')).
*/
:- rdf_meta create_object(r,?),
			object_at(r,r,r,?),
			object_at_table(?),
			object_of_type(r,?),
			test_belief,
			spawn_on_table.

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Interface predicates %%

object_at_table(Instance) :-
	object_at(_, ['map', _, [-1,1,0.8],[0,0,0,1]], 0.4, Instance).

object_of_type(ObjectType, Instance) :-
	belief_existing_objects(Instance, [ObjectType]).

%% Interface predicates %%
%%%%%%%%%%%%%%%%%%%%%%%%%%


table_transform :- 
	[_, _, [0,0,2],[0,0,0,1]].

create_object(ObjectType, Instance):-
	owl_subclass_of(ObjectType, knowrob:'EnduringThing-Localized'),
	belief_new_object(ObjectType, Instance).

object_at(ObjectType, Transform, Threshold, Instance):-
	belief_perceived_at(ObjectType, Transform, Threshold, Instance).



spawn_on_table :-
	belief_perceived_at(knowrob:'Cup', ['map', _, [1,0,0.8],[0,0,0,1]], 0.2, _).


test_belief :-
	belief_perceived_at(knowrob:'Cup', ['map', _, [1,0,0.8],[0,0,0,1]], 0.2, _),
	object_at_table(Instance),
	object_of_type(knowrob:'Cup', Instances),
	member(Instance,Instances),
	clear.

clear :-
	belief_forget.