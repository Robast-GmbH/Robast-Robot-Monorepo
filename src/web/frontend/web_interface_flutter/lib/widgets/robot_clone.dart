import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';

class RobotClone extends StatefulWidget {
  const RobotClone({
    super.key,
    this.onPressed,
    this.selectedDrawerID,
    this.displayStatus = false,
  });
  final void Function(int)? onPressed;
  final int? selectedDrawerID;
  final bool displayStatus;

  @override
  State<RobotClone> createState() => _RobotCloneState();
}

class _RobotCloneState extends State<RobotClone> {
  final openColor = Colors.green; //Color.fromARGB(184, 165, 165, 165);
  final closedColor = Colors.blue; //const Color.fromRGBO(0, 155, 155, 1);

  Color getModuleLedColor(int id) {
    final module = Provider.of<RobotProvider>(context, listen: false).modules["RB0"]?.firstWhere((element) => element.moduleID == id);
    if (module != null && module.status == "open") {
      return openColor;
    }
    return closedColor;
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: (context, constraints) {
      double size = constraints.maxHeight / 1350;
      return Padding(
        padding: const EdgeInsets.all(8.0),
        child: Container(
          width: 570.44 * size,
          height: 1310.10 * size,
          decoration: BoxDecoration(
            border: Border.all(width: 1, strokeAlign: BorderSide.strokeAlignCenter),
            borderRadius: const BorderRadius.vertical(
              top: Radius.circular(8),
            ),
          ),
          child: Column(
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
                            decoration: BoxDecoration(borderRadius: BorderRadius.circular(4), color: Colors.black),
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
                                decoration: BoxDecoration(border: Border.all(), shape: BoxShape.circle, color: Colors.grey),
                                child: Container(
                                  width: 80 * size,
                                  height: 80 * size,
                                  decoration: const BoxDecoration(shape: BoxShape.circle, color: Colors.black),
                                ),
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
                              color: Colors.black,
                              border: Border.all(),
                            ),
                            height: (229.5 - 9.4 - 45.98) * size,
                            child: Container(
                              color: Colors.blue,
                              margin: EdgeInsets.symmetric(vertical: 18.7 * size, horizontal: 19.7 * size),
                              child: Center(
                                  child: Image.asset(
                                "assets/team.png",
                                height: 120 * size,
                              )),
                            ),
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
                  return Column(
                    children: modules.values.first
                        .map((e) => buildDrawer(
                            id: e.moduleID,
                            size: size,
                            height: e.size * 10.45 * size,
                            color: widget.displayStatus ? getModuleLedColor(e.moduleID) : Colors.blue))
                        .toList(),
                  );
                },
              ),
              SizedBox(
                height: (283 / 2 - 43.6) * size,
                width: 570.44 * size,
                child: Center(
                  child: Container(
                    width: 100 * size,
                    height: 25 * size,
                    decoration: BoxDecoration(border: Border.all(width: 0.2), borderRadius: BorderRadius.circular(8)),
                  ),
                ),
              ),
              Container(
                height: 38 * size,
                width: 570.44 * size,
                decoration: BoxDecoration(border: Border.all(width: 0.2)),
              ),
              SizedBox(
                height: (283 / 2 - 43.6) * size,
                width: 570.44 * size,
                child: Center(
                  child: Container(
                    width: 150 * size,
                    height: 60 * size,
                    decoration: BoxDecoration(border: Border.all(width: 0.2), borderRadius: BorderRadius.circular(8)),
                  ),
                ),
              ),
            ],
          ),
        ),
      );
    });
  }

  Widget buildDrawer({required int id, required double size, required double height, required Color color}) {
    final double drawerWidth = 400 * size;
    final double topGap = 20 * size;
    final double bottomGap = 6.5 * size;
    final isSelected = widget.selectedDrawerID == id;
    return GestureDetector(
      onTap: () => widget.onPressed?.call(id),
      child: Container(
        decoration: BoxDecoration(border: Border.all(width: 0.5, strokeAlign: BorderSide.strokeAlignOutside)),
        height: height,
        width: double.infinity,
        child: Column(
          children: [
            SizedBox(
              height: topGap,
            ),
            Expanded(
              child: Container(
                width: drawerWidth,
                decoration: BoxDecoration(
                    border: Border.all(
                      width: 0.5,
                    ),
                    color: Colors.white,
                    boxShadow: isSelected
                        ? const [
                            BoxShadow(
                              color: Colors.grey, // Shadow color
                              offset: Offset(0, 0), // Changes position of shadow
                              blurRadius: 5, // Changes size of shadow
                              spreadRadius: 5, // Expands the shadow
                            ),
                          ]
                        : null,
                    borderRadius: BorderRadius.circular(8)),
                child: Stack(
                  alignment: Alignment.topCenter,
                  children: [
                    Column(
                      children: [
                        SizedBox(
                          height: 25 * size,
                        ),
                        buildGrip(size: size),
                      ],
                    ),
                    Container(
                      width: drawerWidth - 30,
                      height: 2 * size,
                      decoration: BoxDecoration(
                        color: color,
                        boxShadow: [
                          BoxShadow(
                            color: color, // Shadow color
                            offset: const Offset(1, 1), // Changes position of shadow
                            blurRadius: 3, // Changes size of shadow
                            spreadRadius: 1, // Expands the shadow
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
            SizedBox(
              height: bottomGap,
            )
          ],
        ),
      ),
    );
  }

  Widget buildGrip({required double size}) {
    final double width = (400 - 2 * 15.51) * size;
    final double height = 20.56 * size;
    return Container(
      width: width,
      height: height,
      decoration: BoxDecoration(border: Border.all(width: 1), borderRadius: BorderRadius.circular(8)),
    );
  }
}
