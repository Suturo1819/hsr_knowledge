
:- module(object_state,
    [
      create_object/2,
      object_at/4,
      test_belief/1,
      clear/0
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
			test_belief(?).

create_object(ObjectType, Instance):-
	owl_subclass_of(ObjectType, knowrob:'EnduringThing-Localized'),
	belief_new_object(ObjectType, Instance).

object_at(ObjectType, Transform, Threshold, Instance):-
	belief_perceived_at(ObjectType, Transform, Threshold, Instance).

test_belief(Instance):-
	belief_perceived_at(knowrob:'Cup', ['map', _, [0,0,2],[0,0,0,1]], 0.2, Instance).

clear :-
	belief_forget.