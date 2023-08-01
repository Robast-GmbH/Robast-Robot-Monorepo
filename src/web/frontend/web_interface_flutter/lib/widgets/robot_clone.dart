import 'package:flutter/material.dart';

class RobotClone extends StatelessWidget {
  const RobotClone({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: AspectRatio(
        aspectRatio: 55 / 140,
        child: Column(
          children: [
            Expanded(
              flex: 4,
              child: Container(
                decoration: BoxDecoration(
                  border: Border.all(width: 0.2),
                  borderRadius: BorderRadius.only(
                    topLeft: Radius.circular(8),
                    topRight: Radius.circular(8),
                  ),
                ),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.end,
                  children: [
                    Expanded(
                      child: Column(
                        children: [
                          const Expanded(
                            child: SizedBox(),
                          ),
                          Expanded(
                            child: Padding(
                              padding: const EdgeInsets.all(16),
                              child: Container(
                                decoration: const BoxDecoration(
                                  shape: BoxShape.circle,
                                  color: Colors.black,
                                ),
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    Expanded(
                      flex: 2,
                      child: Container(
                        margin: const EdgeInsets.all(16),
                        decoration: BoxDecoration(
                          border: Border.all(width: 16),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: const Center(
                            child: Icon(
                          Icons.landscape,
                          size: 64,
                        )),
                      ),
                    )
                  ],
                ),
              ),
            ),
            buildDrawer(flex: 2),
            buildDrawer(flex: 2),
            buildDrawer(flex: 2),
            buildDrawer(flex: 4),
            buildDrawer(flex: 6),
            Container(
              height: 2,
              width: double.infinity,
              decoration: BoxDecoration(border: Border(left: BorderSide(), right: BorderSide()), color: Colors.green),
            ),
            Expanded(
              flex: 2,
              child: Container(
                decoration: BoxDecoration(border: Border.all(width: 0.2)),
              ),
            ),
            Expanded(
              child: Container(
                decoration: BoxDecoration(border: Border.all(width: 0.2)),
              ),
            ),
            Expanded(
              flex: 2,
              child: Container(
                decoration: BoxDecoration(border: Border.all(width: 0.2)),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Expanded buildDrawer({required int flex}) {
    return Expanded(
      flex: flex,
      child: Container(
        decoration: BoxDecoration(border: Border.all(width: 0.2)),
        child: Container(
          margin: const EdgeInsets.only(top: 16, left: 68, right: 68, bottom: 8),
          decoration: BoxDecoration(
            border: Border.all(),
            borderRadius: BorderRadius.circular(8),
          ),
          child: buildGrip(),
        ),
      ),
    );
  }

  Column buildGrip() {
    return Column(
      children: [
        const SizedBox(
          height: 8,
        ),
        AspectRatio(
          aspectRatio: 17,
          child: Container(
            margin: EdgeInsets.symmetric(horizontal: 8),
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(8),
              border: Border.all(),
            ),
          ),
        ),
      ],
    );
  }
}
