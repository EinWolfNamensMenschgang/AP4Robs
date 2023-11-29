//Laser
string ls_r () {
    /* auslesen der Daten aus der TCP connection über die subscriber*/
};
double* ranges_from_string(string ls_r) {
    /*extrahieren der ranges aus dem ausgelesenen msg string in ein array*/
};

//ODOMETRY
string odo_r () {
    /* auslesen der Daten aus der TCP connection über die subscriber*/
};
odometry_struct odometry_from_string(string msg) {
    /*umwandeln des odometry string msg in ein struct zum weiterverarbeiten*/
};


//Funktionen
twist_struct follow_walls(double* ranges) {
    /*struct für das berechnen der Geschwindigkeiten beim linearen fahren zwischen den Wänden*/
};

void twist_publisher(twist_struct) {
    /*berechnet radiale und lineare geschwindigkeit die benötigt wird*/
};

bool wall_reached(double* ranges) {
    /*returns true/false ob in der gegebenen Reichweite zu einer Wand im Winkel von -5 bis 5°*/
};

twist_struct zylinder(double* ranges) {
    /*struct für das berechnen der Geschwindigkeiten beim radialen fahren um den Zylinder*/
};

void turn_zylinderpunkt(odometry_struct odom, double angle) {
    /*publishing beim ersten ankommen/abfahren vom Zylinder solang differenz zwischen odometry winkel beim ankommen +-90° (=angle) >= 0 twist in bestimmter radialer geschwindigkeit */
};

void speicher_sperren(/*???*/) {
    /*sperren des inputs der subscriber mittels semaphore während Funktionen darauf zugreifen, damit währenddessen die inputs nicht überschrieben werden können*/
};
void speicher_freigeben(/*???*/) {
    /*freigeben des inputs der subscriber mittels Semaphore*/
};
