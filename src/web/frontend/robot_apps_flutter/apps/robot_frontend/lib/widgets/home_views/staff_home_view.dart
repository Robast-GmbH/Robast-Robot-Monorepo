import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/data/svgs.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/content_distribution_task_creation_page.dart';
import 'package:robot_frontend/pages/delivery_task_creation_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/menu_modules_overview.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
import 'package:robot_frontend/widgets/weather_view.dart';
import 'package:robot_frontend/widgets/welcome_view.dart';

class StaffHomeView extends StatelessWidget {
  const StaffHomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          flex: 2,
          child: Column(
            children: [
              Expanded(
                child: WelcomeView(),
              ),
              SizedBox(height: 16),
              Expanded(
                child: Row(
                  children: [
                    Expanded(
                      child: WeatherView(),
                    ),
                    SizedBox(
                      width: 16,
                    ),
                    Expanded(
                        child: CustomButtonView(
                      text: 'Auftrag erstellen',
                      onPressed: () {},
                      content: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          SizedBox(height: 16),
                          Expanded(
                            child: RoundedButton(
                              onPressed: () {
                                Navigator.push(context, MaterialPageRoute<DeliveryTaskCreationPage>(builder: (context) => DeliveryTaskCreationPage()));
                              },
                              child: Text(
                                "Abholen und Abliefern",
                                style: TextStyle(fontSize: 32, color: Colors.white70),
                              ),
                              color: Colors.black.withOpacity(0.1),
                            ),
                          ),
                          SizedBox(height: 16),
                          Expanded(
                            child: RoundedButton(
                              onPressed: () {
                                Navigator.push(context,
                                    MaterialPageRoute<ContentDistributionTaskCreationPage>(builder: (context) => ContentDistributionTaskCreationPage()));
                              },
                              child: Text(
                                "Beladen und Abliefern",
                                style: TextStyle(fontSize: 32, color: Colors.white70),
                              ),
                              color: Colors.black.withOpacity(0.1),
                            ),
                          ),
                          SizedBox(height: 16),
                          Expanded(
                            child: RoundedButton(
                              onPressed: () {},
                              child: Text(
                                "Benutzerdefiniert",
                                style: TextStyle(fontSize: 32, color: Colors.white70),
                              ),
                              color: Colors.black.withOpacity(0.1),
                            ),
                          )
                        ],
                      ),
                    )),
                  ],
                ),
              )
            ],
          ),
        ),
        SizedBox(width: 16),
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: MenuModulesOverview(),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
