import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class NumPad extends StatelessWidget {
  NumPad({this.controller, super.key});

  final TextEditingController? controller;

  final layout = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['CE', '0', 'C'],
  ];

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(16),
      child: Column(
        children: List.generate(
          layout.length,
          (rowIndex) {
            return Expanded(
              child: Column(
                children: [
                  if (rowIndex != 0)
                    Container(
                      color: Colors.black.withOpacity(0.8),
                      height: 1,
                    ),
                  Expanded(
                    child: Row(
                      children: List.generate(
                        layout[rowIndex].length,
                        (columnIndex) {
                          final button = layout[rowIndex][columnIndex];
                          return Expanded(
                            child: Row(
                              children: [
                                if (columnIndex != 0)
                                  Container(
                                    color: Colors.black.withOpacity(0.8),
                                    width: 1.5,
                                  ),
                                Expanded(
                                  child: InkWell(
                                    onTap: () {
                                      if (controller == null) return;
                                      if (button == 'CE') {
                                        controller!.clear();
                                      } else if (button == 'C' && controller!.text.isNotEmpty) {
                                        controller!.text = controller!.text.substring(0, controller!.text.length - 1);
                                      } else if (button != 'C') {
                                        controller!.text += button;
                                      }
                                    },
                                    child: Container(
                                      decoration: BoxDecoration(
                                        color: Colors.black.withOpacity(0.5),
                                      ),
                                      child: Center(
                                        child: Text(
                                          button,
                                          style: const TextStyle(color: RobotColors.primaryText, fontSize: 32),
                                        ),
                                      ),
                                    ),
                                  ),
                                ),
                              ],
                            ),
                          );
                        },
                      ).toList(),
                    ),
                  ),
                ],
              ),
            );
          },
        ).toList(),
      ),
    );
  }
}
