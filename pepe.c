// controlador.c
#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define TIME_STEP 64
#define GRID_SIZE 8
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 4.0
#define MAX_TURNS 100

// Objetivo en coordenadas del mundo (X, Z)
#define GOAL_WORLD_X -2.0
#define GOAL_WORLD_Y -2.0

typedef struct { int x, y; } Point;
typedef struct { int x, y, g, h, f, parent_index; } Node;

int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES]; int open_count = 0;
  Node closed_list[GRID_SIZE*GRID_SIZE]; int closed_count = 0;
  start.g = 0; start.h = heuristic(start.x,start.y,goal.x,goal.y);
  Node start_node = {start.x, start.y, 0, start.h, start.h, -1};
  open_list[open_count++] = start_node;

  while(open_count>0) {
    // extraer mejor f
    int best = 0;
    for(int i=1;i<open_count;i++) if(open_list[i].f<open_list[best].f) best=i;
    Node cur = open_list[best];
    open_list[best] = open_list[--open_count];
    closed_list[closed_count++] = cur;
    if(cur.x==goal.x && cur.y==goal.y) {
      // reconstruir camino
      int len=0;
      Node node = cur;
      while(node.parent_index!=-1 && len<max_path_len) {
        path[len++] = (Point){node.x,node.y};
        node = closed_list[node.parent_index];
      }
      path[len++] = (Point){cur.x,cur.y};
      // invertir
      for(int i=0;i<len/2;i++) {
        Point t=path[i]; path[i]=path[len-1-i]; path[len-1-i]=t;
      }
      return len;
    }
    const int dx[4]={0,1,0,-1};
    const int dy[4]={1,0,-1,0};
    for(int d=0;d<4;d++) {
      int nx=cur.x+dx[d], ny=cur.y+dy[d];
      if(nx<0||ny<0||nx>=GRID_SIZE||ny>=GRID_SIZE) continue;
      if(grid[nx][ny]==1) continue;
      // si cerrada
      bool closed=false;
      for(int i=0;i<closed_count;i++) if(closed_list[i].x==nx && closed_list[i].y==ny) {closed=true;break;}
      if(closed) continue;
      int g=cur.g+1;
      int h=heuristic(nx,ny,goal.x,goal.y);
      int f=g+h;
      // buscar en open
      bool in_open=false;
      for(int i=0;i<open_count;i++) {
        if(open_list[i].x==nx && open_list[i].y==ny) {
          in_open=true;
          if(f<open_list[i].f) {
            open_list[i].g=g; open_list[i].h=h; open_list[i].f=f; open_list[i].parent_index=closed_count-1;
          }
          break;
        }
      }
      if(!in_open && open_count<MAX_OPEN_NODES) {
        open_list[open_count++] = (Node){nx,ny,g,h,f,closed_count-1};
      }
    }
  }
  return 0;
}

int main() {
  wb_robot_init();
  // inicializar dispositivos
  WbDeviceTag wheels[4]; const char *wn[4]="wheel1 wheel2 wheel3 wheel4";
  for(int i=0;i<4;i++){ wheels[i]=wb_robot_get_device(strtok((char*)wn," ")); wb_motor_set_position(wheels[i],INFINITY); wb_motor_set_velocity(wheels[i],0); }
  WbDeviceTag ds[2]; const char *dsn[2]="ds_left ds_right";
  for(int i=0;i<2;i++){ ds[i]=wb_robot_get_device(strtok((char*)dsn," ")); wb_distance_sensor_enable(ds[i],TIME_STEP); }
  WbDeviceTag lidar=wb_robot_get_device("lidar"); wb_lidar_enable(lidar,TIME_STEP); wb_lidar_enable_point_cloud(lidar);
  WbDeviceTag gps=wb_robot_get_device("gps"); wb_gps_enable(gps,TIME_STEP);

  int grid[GRID_SIZE][GRID_SIZE] = {{0}};
  Point path[MAX_PATH_LEN]; int path_len=0;
  double origin_x=0, origin_z=0; bool init=false, reached=false;
  int goal_x_cell=0, goal_z_cell=0;
  char turn_log[MAX_TURNS][20]; int turn_count=0;
  double last_l=0, last_r=0;

  while(wb_robot_step(TIME_STEP)!=-1) {
    const double *pos = wb_gps_get_values(gps);
    double rx=pos[0], rz=pos[2];
    if(!init) {
      origin_x=rx; origin_z=rz;
      goal_x_cell = (int)((GOAL_WORLD_X-origin_x+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
      goal_z_cell = (int)((GOAL_WORLD_Y-origin_z+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
      goal_x_cell = fmax(0, fmin(GRID_SIZE-1,goal_x_cell));
      goal_z_cell = fmax(0, fmin(GRID_SIZE-1,goal_z_cell));
      init=true;
    }
    int cx=(int)((rx-origin_x+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
    int cz=(int)((rz-origin_z+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
    cx=fmax(0,fmin(GRID_SIZE-1,cx)); cz=fmax(0,fmin(GRID_SIZE-1,cz));
    printf("Robot en celda:(%d,%d)\n",cx,cz);

    if(!reached && cx==goal_x_cell && cz==goal_z_cell) {
      printf("\n¡Llegó a la meta en celda(%d,%d)!\n",goal_x_cell,goal_z_cell);
      printf("Secuencia de giros:\n"); for(int i=0;i<turn_count;i++) printf("%s\n",turn_log[i]);
      for(int i=0;i<4;i++) wb_motor_set_velocity(wheels[i],0);
      break;
    }
    // limpiar grid
    memset(grid,0,sizeof(grid));
    const float *ranges = wb_lidar_get_range_image(lidar);
    int res=wb_lidar_get_horizontal_resolution(lidar);
    double fov=wb_lidar_get_fov(lidar);
    bool obs_close=false;
    for(int i=0;i<res;i++){
      double angle=-fov/2 + i*(fov/res);
      double d=ranges[i]; if(isinf(d)||d>1.0) continue;
      double ox=rx + d*cos(angle), oz=rz + d*sin(angle);
      int gx=(int)((ox-origin_x+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
      int gz=(int)((oz-origin_z+GRID_SIZE*CELL_SIZE/2)/CELL_SIZE);
      if(gx>=0&&gx<GRID_SIZE&&gz>=0&&gz<GRID_SIZE) grid[gx][gz]=1;
      if(d<0.3) obs_close=true;
    }
    // vecinos ultrasónicos
    double ds0=wb_distance_sensor_get_value(ds[0]);
    double ds1=wb_distance_sensor_get_value(ds[1]);
    if(ds0<950||ds1<950) obs_close=true;

    Point start={cx,cz}; Point goal={goal_x_cell,goal_z_cell};
    path_len=plan_path(grid,start,goal,path,MAX_PATH_LEN);

    double ls=SPEED, rs=SPEED;
    if(obs_close) { ls=1; rs=-1; }
    // log de giro
    if(turn_count<MAX_TURNS) {
      if(ls==1&&rs==-1&&!(last_l==1&&last_r==-1)) strcpy(turn_log[turn_count++],"izquierda");
      if(ls==-1&&rs==1&&!(last_l==-1&&last_r==1)) strcpy(turn_log[turn_count++],"derecha");
    }
    last_l=ls; last_r=rs;
    for(int i=0;i<4;i++) wb_motor_set_velocity(wheels[i], i%2?rs:ls);
    fflush(stdout);
  }
  wb_robot_cleanup();
  return 0;
}