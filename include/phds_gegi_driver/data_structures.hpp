//
// Created by createc on 09/09/2020.
//

#ifndef PHDS_GEGI_DRIVER_DATA_STRUCTURES_HPP
#define PHDS_GEGI_DRIVER_DATA_STRUCTURES_HPP

namespace phds_gegi_driver {
    struct Point3 {
        double x;
        double y;
        double z;
    };

    namespace compton_events {

        struct Interaction {
            double avg_energy;
            Point3 site;
        };

        struct Event1Site {
            double global_time;

            Interaction compton_interaction;

        };

        struct Event2Site {
            double global_time;

            Interaction compton_interaction_1;

            Interaction compton_interaction_2;

            double compton_angle;
            double delta_compton_angle;

        };

    }
}

#endif //PHDS_GEGI_DRIVER_DATA_STRUCTURES_HPP
