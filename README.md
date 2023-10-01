# projet-dvrp-cpp

Dynamic Vehicle Routing with Ant Colony Optimization

# Classes

## Node

Représentation d'un client ou d'un dépot.

### Membres

- (u_int) id
- (float) x, y : position
- (bool) is_depot
- (float) available_time (0 si c'est un dépot)
- (int) demand (0 si c'est un dépot)
- (float) service_time : temps requis pour satisfaire la commande en plus du trajet (0 si c'est un dépot)

### Précisions

Les id des noeuds sont attribués ainsi :
{1, ..., N} pour les noeuds clients (N est le nombre de clients)
{N + 1, ..., N + V} pour les noeuds véhicules (V est le nombre de véhicules)

## TourAtom

Une solution est représentée par un vector de TourAtom.
Les attributs des TourAtom composant une solution sont accumulés tant que le véhicule n'est pas retourné au dépot

### Membres

- (u_int) node_id
- (float) load : place occupée dans le véhicule
- (float) end_of_service : temps quand le dernier noeud a été servi (incluant le service_time)
- (float) distance : distance totale parcourue depuis le dépot

### Précisions

Les attributs des TourAtom composant une solution sont cumulés tout au long du trajet d'un véhicule jusqu'à ce que ce dernier retourne au dépot.

Exemple de solution
(node_id, load, end_of_service, distance)

(106, 0, 0, 0) // Reset load, end_of_service et distance à zéro

(20, 10, 20.2265, 2.02265)

(24, 20, 39.4417, 3.03398)

(25, 60, 58.0502, 3.43851)

(28, 80, 76.9833, 4.16779)

(101, 0, 0, 0) // Reset load, end_of_service et distance à zéro

(67, 10, 20.6728, 2.46896)

(66, 20, 39.8881, 3.48029)

(68, 30, 59.1812, 4.56952)

(102, 0, 0, 0) // Reset load, end_of_service et distance à zéro

(5, 10, 21.2647, 3.06083)

(2, 40, 40.648, 4.24023)

## Problem

Cette classe contient toutes les données du problème.
Ses getters sont appelés par main, ant_colony et ant

### Membres

- vector<Node \*> nodes : stocke les noeuds de tous les clients du dataset (même ceux pas encore disponibles) et les noeuds des véhicules
- vector<u_int> available_nodes_ids : identifiants des noeuds disponibles depuis le dernier update (contient également les identifiants des noeuds de type dépot)
- available_c_nodes_ids : comme available_nodes_ids mais ne contient que les noeuds de type client
- map< u_int, vector<u_int> > vehicles_commitments : associe à chaque véhicule les clients qui lui ont été assignés (on y accède par vehicule_number pas par node_id)
- vector<u_int> committed_c_nodes_ids : identifiants de tous les noeuds qui ont déjà été assignés
- float last_update_time : temps (depuis le début de la journée) où la fonction update a été appellée pour la dernière fois

- constructeur : lit le jeu de données et construit les membres

## AntColony

Classe qui gère l'optimisation par colonie de fourmi.
Contient des données (meilleure solution, matrice de phéromone)

### Membres

- () -> () step : fonction qui exécute une itération de l'optimisation c-à-d instanciation de num_ants fourmies, construction des solutions, mise à jour locale et globale de la matrice de phéromone. Si une solution meilleure que la solution actuelle est trouvée, elle est acceptée
- () -> () update_solution : fonction qui doit être appellée uniquement si de nouveaux noeuds sont disponibles (après un appel à Problem::update). Elle va instancier une fourmi et accepter comme meilleure solution la première solution qui est trouvée par cette fourmi

## Ant

Classe qui gère la construction d'une solution potentielle par une fourmi

### Membres

- vector<u_int> visited_nodes : les id des noeuds visités depuis le début de la construction de la solution
- u_int num_visited_customers : le nombre de clients (on ne compte pas les dépots) visités depuis le début de la construction
- (u_int) current_node_id : id du noeud actuel
- (u_int) current_vehicle_number : numéro du véhicule actuel (dans {1, ..., V})
- current_time, current_load, current_distance : les attributs sont accumulés jusqu'à ce que la fourmi retourne au dépot

- void initialize_tour() : choisit aléatoirement un dépot et initialise current_node_id, etc. De plus cette méthode appelle Ant::insert_committed_customers pour ajouter les clients déjà assignés au véhicule choisit aléatoirement.

* compute_candidate_arcs() : détermine quels noeuds sont visitables à partir de la situation actuelle (current_load, current_node_id). On part de tous les noeuds disponibles et on en retire au fur et à mesure. On enlève les noeuds déjà visités, les noeuds pour lesquels la capacité n'est pas suffisante, les noeuds déjà assignés, et si le véhicule est à un dépot alors on enlève tous les dépots

* select_arc_nn/acs() : à partir des noeuds visitables, on en choisit un soit le plus proche, soit un noeud calculés d'après les articles

* insert_selected_arc() : on insère le noeud sélectionné. Cette fonction met à jour current_time, current_load etc. De plus si le noeud ajouté est un dépot alors il faut ajouter tous ses clients déjà assignés en appelant Ant::insert_committed_customers.

* insert_committed_customers(u_int vehicle_number) : on insère les noeuds clients déjà assignés au véhicule. À chaque fois qu'un client est inséré, on incrément num_visited_customers.


## Local_search

Classe qui fait la recherche locale sur une solution

### Membres
- Problem : contient toutes les données du problème
- vector<TourAtom> solution : solution du vehicle routing problem

- search(int t_ls) : Lance la recherche. La recherche sera arrêtée après t_ls secondes si elle n'pas terminé.
- compute_solution_score(vector<TourAtom> solution) : calcul le scrore de la solution amélioré 
- solution_from_search() : retourne la solution amélioré 