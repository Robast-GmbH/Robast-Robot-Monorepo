import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/constants/gaps.dart';
import 'package:web_interface_flutter/models/team_member.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';

class TeamPage extends StatelessWidget {
  TeamPage({super.key});
  final scrollController = ScrollController();
  final ourTeam = [
    TeamMember(imagePath: "assets/team_members/tobi.png", name: "Tobias", role: "Softwareentwickler/Geschäftsführung"),
    TeamMember(imagePath: "assets/team_members/jacob.png", name: "Jacob", role: "Softwareentwickler"),
    TeamMember(imagePath: "assets/team_members/miriam.png", name: "Miriam", role: "Robotik Ingenieurin"),
    TeamMember(imagePath: "assets/team_members/sagar.png", name: "Sagar", role: "Softwareentwickler"),
    TeamMember(imagePath: "assets/team_members/simon.png", name: "Simon", role: "Produktentwickler"),
    TeamMember(imagePath: "assets/team_members/andreas.png", name: "Andreas", role: "Softwareentwickler"),
  ];

  @override
  Widget build(BuildContext context) {
    return PageFrame(
      title: "Unser Team",
      color: AppColors.darkBlue,
      child: Container(
        margin: Constants.mediumPadding,
        decoration: BoxDecoration(
          color: AppColors.lightGrey,
          borderRadius: BorderRadius.circular(16),
        ),
        padding: Constants.largePadding,
        child: Scrollbar(
          controller: scrollController,
          thumbVisibility: true,
          child: GridView.count(
            controller: scrollController,
            childAspectRatio: 3,
            shrinkWrap: true,
            crossAxisCount: kIsWeb ? 1 : 2,
            children: ourTeam
                .map(
                  (e) => Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Container(
                        decoration: const BoxDecoration(shape: BoxShape.circle),
                        child: Image.asset(e.imagePath),
                      ),
                      Gaps.mediumHorizontal,
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(top: 32),
                              child: Text(
                                e.name,
                                style: const TextStyle(fontSize: 24),
                              ),
                            ),
                            Padding(
                              padding: const EdgeInsets.only(top: 8),
                              child: Text(
                                e.role,
                                style: const TextStyle(fontSize: 20),
                              ),
                            ),
                          ],
                        ),
                      )
                    ],
                  ),
                )
                .toList(),
          ),
        ),
      ),
    );
  }
}
