
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
  ////////  ///     //////// //      //   //           //         //  //////  ///   // //    //   //////  ///    '''

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
    def PrintSeparator():
        print(f"{font.BOLD} ===================================================================================================== {font.END}")


if __name__ == '__main__':
    Plotter.PlotMonitorBanner()
    Plotter.PlotClientBanner()
