import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/data/drawer_module.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';

class RobotClone extends StatelessWidget {
  const RobotClone({
    super.key,
    this.onPressed,
    this.displayStatus = false,
    this.selectedModule,
  });
  final void Function(int)? onPressed;
  final bool displayStatus;
  final int? selectedModule;

  Color getModuleLedColor(DrawerModule module) {
    if ((module.status == "Opened" && displayStatus) || selectedModule == module.moduleID) {
      return AppColors.green;
    }
    return AppColors.blue;
  }

  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return LayoutBuilder(builder: (context, constraints) {
      double size = constraints.maxHeight / 1400;
      return Stack(
        alignment: Alignment.bottomCenter,
        children: [
          ClipOval(
            child: Container(
              width: size * 750,
              height: size * 200,
              color: AppColors.grey,
            ),
          ),
          Padding(
            padding: Constants.smallPadding,
            child: Column(
              children: [
                Container(
                  width: 570.44 * size,
                  height: 1310.10 * size,
                  decoration: BoxDecoration(
                    color: AppColors.white,
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.start,
                    crossAxisAlignment: CrossAxisAlignment.center,
                    children: [
                      SizedBox(
                        width: (570.44 - 40.46 - 35.80) * size,
                        height: 229.50 * size,
                        child: Row(
                          children: [
                            SizedBox(
                              width: (237.23 - 40.46) * size,
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  SizedBox(
                                    height: 19.8 * size,
                                  ),
                                  Container(
                                    width: 97 * size,
                                    height: 29.5 * size,
                                    decoration: BoxDecoration(borderRadius: BorderRadius.circular(4), color: AppColors.grey),
                                  ),
                                  SizedBox(
                                    height: 40.54 * size,
                                  ),
                                  Row(
                                    children: [
                                      SizedBox(
                                        width: (55.15 - 40.45) * size,
                                      ),
                                      Container(
                                        padding: EdgeInsets.all(6 * size),
                                        width: 100 * size,
                                        height: 100 * size,
                                        decoration: const BoxDecoration(shape: BoxShape.circle, color: AppColors.grey),
                                      )
                                    ],
                                  )
                                ],
                              ),
                            ),
                            SizedBox(
                              width: (570.44 - 237.23 - 35.80) * size,
                              child: Column(
                                children: [
                                  SizedBox(
                                    height: 9.4 * size,
                                  ),
                                  Container(
                                    decoration: BoxDecoration(
                                      color: AppColors.grey,
                                      borderRadius: BorderRadius.circular(4),
                                    ),
                                    height: (229.5 - 9.4 - 45.98) * size,
                                  )
                                ],
                              ),
                            ),
                          ],
                        ),
                      ),
                      Selector<RobotProvider, Map<String, List<DrawerModule>>>(
                          selector: (_, provider) => provider.modules,
                          builder: (context, modules, child) {
                            if (modules.isEmpty) return const SizedBox();
                            return Column(
                              children: robotProvider.modules.values.first
                                  .map(
                                    (e) => buildDrawer(
                                      drawer: e,
                                      height: e.size * 10.45 * size *10,
                                    ),
                                  )
                                  .toList(),
                            );
                          }),
                      SizedBox(
                        height: (283 / 2 - 43.6) * size,
                        width: 570.44 * size,
                        child: Center(
                          child: Container(
                            width: 100 * size,
                            height: 25 * size,
                            decoration: BoxDecoration(
                              color: AppColors.grey,
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        ),
                      ),
                      Container(
                        margin: const EdgeInsets.symmetric(horizontal: 4),
                        height: 38 * size,
                        width: 570.44 * size,
                        decoration: const BoxDecoration(color: AppColors.grey),
                      ),
                      SizedBox(
                        height: (283 / 2 - 43.6) * size,
                        width: 570.44 * size,
                        child: Center(
                          child: Container(
                            width: 150 * size,
                            height: 60 * size,
                            decoration: BoxDecoration(
                              color: AppColors.grey,
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Container(
                      height: size * 20,
                      width: size * 140,
                      decoration: const BoxDecoration(
                        color: AppColors.lightGrey,
                        borderRadius: BorderRadius.vertical(
                          bottom: Radius.circular(16),
                        ),
                      ),
                    ),
                    SizedBox(
                      width: size * 260,
                    ),
                    Container(
                      height: size * 20,
                      width: size * 140,
                      decoration: const BoxDecoration(
                        color: AppColors.lightGrey,
                        borderRadius: BorderRadius.vertical(
                          bottom: Radius.circular(16),
                        ),
                      ),
                    )
                  ],
                )
              ],
            ),
          ),
        ],
      );
    });
  }

  Widget buildDrawer({
    required DrawerModule drawer,
    required double height,
  }) {
    return GestureDetector(
      onTap: () => onPressed?.call(drawer.moduleID),
      child: Container(
        margin: const EdgeInsets.only(left: 2, right: 2, bottom: 2),
        decoration: BoxDecoration(color: getModuleLedColor(drawer)),
        height: height - 2,
        width: double.infinity,
        child: Padding(
          padding: Constants.smallPadding,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                drawer.label,
                style: const TextStyle(
                  color: AppColors.white,
                  fontWeight: FontWeight.w500,
                  fontSize: kIsWeb ? 15 : 16,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
