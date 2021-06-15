
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

class Plotter(object):

    @staticmethod
    def PlotMonitorBanner():
        banner = '''
   ********                           **                ****     ****                   **   **
  **//////**                  ****** /**               /**/**   **/**                  //   /**
 **      //  ******  ******  /**///**/**               /**//** ** /**  ******  *******  ** ******  ******  ******
/**         //**//* //////** /**  /**/******    *****  /** //***  /** **////**//**///**/**///**/  **////**//**//*
/**    ***** /** /   ******* /****** /**///**  /////   /**  //*   /**/**   /** /**  /**/**  /**  /**   /** /** /
//**  ////** /**    **////** /**///  /**  /**          /**   /    /**/**   /** /**  /**/**  /**  /**   /** /**
 //******** /***   //********/**     /**  /**          /**        /**//******  ***  /**/**  //** //****** /***
  ////////  ///     //////// //      //   //           //         //  //////  ///   // //    //   //////  ///'''

        print(f'\n{banner}\n\n\n')

    @staticmethod
    def PlotClientBanner():
        banner = '''
   ********                           **                  ******   ** **                    **
  **//////**                  ****** /**                 **////** /**//                    /**
 **      //  ******  ******  /**///**/**                **    //  /** **  *****  *******  ******
/**         //**//* //////** /**  /**/******    *****  /**        /**/** **///**//**///**///**/
/**    ***** /** /   ******* /****** /**///**  /////   /**        /**/**/******* /**  /**  /**
//**  ////** /**    **////** /**///  /**  /**          //**    ** /**/**/**////  /**  /**  /**
 //******** /***   //********/**     /**  /**           //******  ***/**//****** ***  /**  //**
  ////////  ///     //////// //      //   //             //////  /// //  ////// ///   //    //  '''

        print(f'\n{banner}\n\n\n')

    @staticmethod
    def PrintMonitorConfig(config):
        print(f'{font.BLUE} --- General Configuration -------------------------------------------------- {font.END}')
        print(f'{font.BOLD} Update rate:{font.END} {1e9/config.rate.sleep_dur.to_nsec()}hz')
        print(f'{font.BOLD} Compute submap-to-submap constraints:{font.END} {config.enable_submap_constraints}')
        print(f'{font.BOLD} Reduce optimized graph using Kron:{font.END} {config.reduce_global_graph}')
        print(f'{font.BOLD} Minimal node count in optimized graph:{font.END} {config.min_node_count}')
        print(f'{font.BOLD} Minimal submap count:{font.END} {config.submap_min_count}')
        print(f'{font.BOLD} Submap pivot distance:{font.END} {config.pivot_distance}')
        print(f'{font.BOLD} Submap k-nearest neighbors:{font.END} {config.n_nearest_neighbors}')
        print(f'{font.BOLD} Submap visualize map:{font.END} {config.enable_submap_map_publishing}')
        print(f'{font.BOLD} Submap compute LiDAR poses:{font.END} {config.compute_poses_in_LiDAR}')
        print(f'{font.BOLD} Submap ICP refinement:{font.END} {config.refine_with_ICP}')
        print('\n')

        print(f'{font.YELLOW} --- Subscriber Configuration (Input) --------------------------------------- {font.END}')
        print(f'{font.BOLD} Optimized graph topic:{font.END} {config.in_graph_topic}')
        print(f'{font.BOLD} Optimized trajectory topic:{font.END} {config.in_traj_opt_topic}')
        print(f'{font.BOLD} Optimized submap topic:{font.END} {config.opt_pc_topic}')
        print(f'{font.BOLD} Verifcation request topic:{font.END} {config.verification_service_topic}')
        print('\n')

        print(f'{font.GREEN} --- Publisher Configuration (Output) --------------------------------------- {font.END}')
        print(f'{font.BOLD} Optimized graph topic:{font.END} {config.out_graph_topic}')
        print(f'{font.BOLD} Optimized trajectory topic:{font.END} {config.out_traj_opt_topic}')
        print(f'{font.BOLD} Optimized submap constraints topic:{font.END} {config.submap_topic}')
        print('\n')

    @staticmethod
    def PrintClientConfig(config):
        print(f'{font.BLUE} --- General Configuration -------------------------------------------------- {font.END}')
        print(f'{font.BOLD} Update rate:{font.END} {1e9/config.rate.sleep_dur.to_nsec()}hz')
        print(f'{font.BOLD} Dataroot:{font.END} {config.dataroot}')
        print(f'{font.BOLD} Path to the random forest model:{font.END} {config.random_forest_model}')
        print(f'{font.BOLD} Robot name:{font.END} {config.robot_name}')
        print(f'{font.BOLD} Enable submap constraints:{font.END} {config.enable_submap_constraints}')
        print(f'{font.BOLD} Enable anchor constraints:{font.END} {config.enable_anchor_constraints}')
        print(f'{font.BOLD} Enable signal recording:{font.END} {config.enable_signal_recording}')
        print(f'{font.BOLD} Enable trajectory recording:{font.END} {config.enable_trajectory_recording}')
        print(f'{font.BOLD} Signal export path:{font.END} {config.signal_export_path}')
        print(f'{font.BOLD} Trajectory export path:{font.END} {config.trajectory_export_path}')
        print('\n')

        print(f'{font.YELLOW} --- Subscriber Configuration (Input) --------------------------------------- {font.END}')
        print(f'{font.BOLD} Optimized graph topic:{font.END} {config.opt_graph_topic}')
        print(f'{font.BOLD} Optimized trajectory topic:{font.END} {config.opt_traj_topic}')
        print(f'{font.BOLD} Estimated trajectory topic:{font.END} {config.est_traj_topic}')
        print(f'{font.BOLD} Estimated trajectory (Path) topic:{font.END} {config.est_traj_path_topic}')
        print(f'{font.BOLD} Client update topic:{font.END} {config.client_update_topic}')
        print('\n')

        print(f'{font.GREEN} --- Publisher Configuration (Output) --------------------------------------- {font.END}')
        print(f'{font.BOLD} Anchor node topic:{font.END} {config.anchor_node_topic}')
        print(f'{font.BOLD} Relative node topic:{font.END} {config.relative_node_topic}')
        print(f'{font.BOLD} Intra constraints topic:{font.END} {config.intra_constraint_topic}')
        print(f'{font.BOLD} Verifcation service topic:{font.END} {config.verification_service_topic}')
        print('\n')

    @staticmethod
    def PrintSeparator():
        print(f"{font.BOLD} ===================================================================================================== {font.END}")


if __name__ == '__main__':
    Plotter.PlotMonitorBanner()
    Plotter.PlotClientBanner()
