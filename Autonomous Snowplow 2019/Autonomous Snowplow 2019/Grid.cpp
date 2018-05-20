#include "Grid.h"

/*Global variables*/
vector<vector<int>> hit_map;
double resolution = 0.25;
lidar_handler* lidar_ref;
lidar_handler::lidar_data_packet* data_packet;
int tot_scans_mapped = 0;

//constructor
Grid::Grid(double width, double height, lidar_handler *lidar) {
	lidar_ref = lidar;
	for (int i = 0; i < width / resolution; i++) {//x axis
		vector<int> temp;
		for (int j = 0; j < height / resolution; j++) {//y axis
			temp.push_back(0);
		}
		hit_map.push_back(temp);
	}
	cout << "done with grid" << endl;
	print_hit_map();
}

void Grid::print_hit_map() {
	//system("cls");
	printf("\n");
	for (int j = 0; j < hit_map.size(); j++) {
		for (int i = hit_map[j].size() - 1; i > 0; i--) {
			printf("%i", hit_map[i][j]);
			string test = to_string(hit_map[i][j]);
			for (int w = 0; w < 4 - test.length(); w++) {
				printf(" ");
			}
			printf(" ");
		}
		printf("\n");
	}
}

//thread running for grid
void Grid::run() {
	while (1) {
		
		update_hit_map();

	}
	//map hits
}

void Grid::update_hit_map() {
	if ((*lidar_ref).data_is_ready()) {
		/*get reference to data*/
		data_packet = (*lidar_ref).get_data();
		//cout << "data points: " << data_packet->data_points << endl;
		//cout << "data is ready to process." << endl;

		for (int i = 0; i < data_packet->data_points; i++) {
			//map each point
			//cout << "i is: " << i << " dp's are: " << data_packet->data_points << endl;

			double angle = get<0>(data_packet->raw_data[i]);
			double distance = get<1>(data_packet->raw_data[i]);

			if (angle < 0) {
				angle = 360 - abs(angle);
			}
			double x_loc = (cos(angle*3.141592 / 180.0) * distance) + (*lidar_ref).get_x_loc();
			double y_loc = (sin(angle*3.141592 / 180.0) * distance) + (*lidar_ref).get_y_loc();
			//cout << "Angle of: " << angle << " x location: " << x_loc << " y location: " << y_loc << endl;
			
			int x_index = floor(x_loc / resolution);
			int y_index = floor(y_loc / resolution);
			if (x_index < 0 || x_index > hit_map.size() - 1 || y_index < 0 || y_index > hit_map[0].size() - 1) {
				continue;
			}

			if (hit_map[floor(x_loc / resolution)][floor(y_loc / resolution)] < 9999) {
				hit_map[floor(x_loc / resolution)][floor(y_loc / resolution)] += 1;
			}

		}

		//cout << "finished processing data." << endl;
		tot_scans_mapped++;
		if (tot_scans_mapped % 110 == 0) {
			print_hit_map();
			cout << endl << "=================================================================================" << endl;
		}
	}
	else {
		//cout << "data isn't ready yet..." << endl;
	}
}
