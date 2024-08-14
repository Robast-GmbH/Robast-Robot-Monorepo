import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class DrawerView extends StatelessWidget {
  const DrawerView({
    required this.module,
    super.key,
    this.showReservationStatus = false,
    this.label,
    this.onPressed,
  });

  final RobotDrawer module;
  final bool showReservationStatus;
  final String? label;
  final VoidCallback? onPressed;

  @override
  Widget build(BuildContext context) {
    final isReserved = module.isReserved();
    return Expanded(
      flex: module.size,
      child: GestureDetector(
        onTap: () {
          onPressed?.call();
        },
        child: Column(
          children: [
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(4),
                child: Container(
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      begin: Alignment.bottomCenter,
                      end: Alignment.topCenter,
                      colors: showReservationStatus && !isReserved
                          ? [
                              Colors.white.withOpacity(0.5),
                              Colors.white.withOpacity(0.3),
                            ]
                          : [
                              const Color(0xCCBBFF33),
                              const Color(0x7FA8E52D),
                            ],
                    ),
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: SizedBox.expand(
                    child: Align(
                      child: Text(
                        label ?? '',
                        textAlign: TextAlign.center,
                        style: const TextStyle(
                          height: 0,
                          color: Colors.white,
                          fontSize: 40,
                          fontWeight: FontWeight.w400,
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
