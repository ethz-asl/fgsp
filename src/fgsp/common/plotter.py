#! /usr/bin/env python3

class font:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'
   CLEAR = '\033c'

class Plotter(object):

    @staticmethod
    def PlotMonitorBanner():
        banner = '''
   ********        ****     ****
  **//////**      /**/**   **/**
 **      //       /**//** ** /**  ******  *******
/**          *****/** //***  /** **////**//**///**
/**    *****///// /**  //*   /**/**   /** /**  /**
//**  ////**      /**   /    /**/**   /** /**  /**
 //********       /**        /**//******  ***  /**
  ////////        //         //  //////  ///   // '''

        print('\n{banner}\n\n\n'.format(banner = banner))

    @staticmethod
    def PlotClientBanner():
        banner = '''
   ********          ******   ** **                    **
  **//////**        **////** /**//                    /**
 **      //        **    //  /** **  *****  *******  ******
/**          *****/**        /**/** **///**//**///**///**/
/**    *****///// /**        /**/**/******* /**  /**  /**
//**  ////**      //**    ** /**/**/**////  /**  /**  /**
 //********        //******  ***/**//****** ***  /**  //**
  ////////          //////  /// //  ////// ///   //    //  '''

        print('\n{banner}\n\n\n'.format(banner = banner))

    @staticmethod
    def PrintMonitorConfig(config):
        print('{color} --- General Configuration -------------------------------------------------- {end}'.format(color=font.BLUE, end=font.END))
        print('{bold} Update rate:{end} {val}hz'.format(bold=font.BOLD, end=font.END, val = 1/config.rate))
        print('{bold} Compute submap-to-submap constraints:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_submap_constraints))
        print('{bold} Reduce optimized graph using Kron:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.reduce_global_graph))
        print('{bold} Minimal node count in optimized graph:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.min_node_count))
        print('{bold} Minimal submap count:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.submap_min_count))
        print('{bold} Submap pivot distance:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.pivot_distance))
        print('{bold} Submap k-nearest neighbors:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.n_nearest_neighbors))
        print('{bold} Submap visualize map:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_submap_map_publishing))
        print('{bold} Submap compute LiDAR poses:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.compute_poses_in_LiDAR))
        print('\n')

        print('{color} --- Subscriber Configuration (Input) --------------------------------------- {end}'.format(color=font.YELLOW, end=font.END))
        print('{bold} Optimized graph topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.in_graph_topic))
        print('{bold} Optimized trajectory topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.in_traj_opt_topic))
        print('{bold} Optimized submap topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.opt_pc_topic))
        print('{bold} Verifcation request topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.verification_service_topic))
        print('\n')

        print('{color} --- Publisher Configuration (Output) --------------------------------------- {end}'.format(color=font.GREEN, end=font.END))
        print('{bold} Optimized graph topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.out_graph_topic))
        print('{bold} Optimized trajectory topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.out_traj_opt_topic))
        print('{bold} Optimized submap constraints topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.submap_topic))
        print('\n')

    @staticmethod
    def PrintClientConfig(config):
        print('{color} --- General Configuration -------------------------------------------------- {end}'.format(color=font.BLUE, end=font.END))
        print('{bold} Update rate:{end} {val}hz'.format(bold=font.BOLD, end=font.END, val = 1/config.rate))
        print('{bold} Dataroot:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.dataroot))
        print('{bold} Robot name:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.robot_name))
        print('{bold} Enable submap constraints:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_submap_constraints))
        print('{bold} Enable anchor constraints:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_anchor_constraints))
        print('{bold} Enable signal recording:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_signal_recording))
        print('{bold} Enable trajectory recording:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.enable_trajectory_recording))
        print('{bold} Signal export path:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.signal_export_path))
        print('{bold} Trajectory export path:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.trajectory_export_path))
        print('\n')

        print('{color} --- Operating Mode --------------------------------------------------------- {end}'.format(color=font.RED, end=font.END))
        if config.use_se3_computation:
            print('{bold} Using SE(3) computations {end}'.format(bold=font.BOLD, end=font.END))
        elif config.use_so3_computation:
            print('{bold} Using SO(3) computations {end}'.format(bold=font.BOLD, end=font.END))
        else:
            print('{bold} Using R^3 computations {end}'.format(bold=font.BOLD, end=font.END))
            print('{bold} Including rotational weight: {end} {val}'.format(bold=font.BOLD, end=font.END, val = config.include_rotational_weight))
            print('{bold} Including temporal decay: {end} {val}'.format(bold=font.BOLD, end=font.END, val = config.include_temporal_decay_weight))
        print('{bold} Classifier: {end} {val}'.format(bold=font.BOLD, end=font.END, val = config.classifier))
        print('\n')

        print('{color} --- Subscriber Configuration (Input) --------------------------------------- {end}'.format(color=font.YELLOW, end=font.END))
        print('{bold} Optimized graph topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.opt_graph_topic))
        print('{bold} Optimized trajectory topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.opt_traj_topic))
        print('{bold} Estimated trajectory topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.est_traj_topic))
        print('{bold} Estimated trajectory (Path) topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.est_traj_path_topic))
        print('{bold} Client update topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.client_update_topic))
        print('\n')

        print('{color} --- Publisher Configuration (Output) --------------------------------------- {end}'.format(color=font.GREEN, end=font.END))
        print('{bold} Anchor node topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.anchor_node_topic))
        print('{bold} Relative node topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.relative_node_topic))
        print('{bold} Intra constraints topic:{end} {val}'.format(bold=font.BOLD, end=font.END, val = config.intra_constraint_topic))
        print('\n')

    @staticmethod
    def PrintSeparator():
        print("{bold} ===================================================================================================== {end}".format(bold=font.BOLD, end=font.END))

if __name__ == '__main__':
    Plotter.PlotMonitorBanner()
    Plotter.PlotClientBanner()
