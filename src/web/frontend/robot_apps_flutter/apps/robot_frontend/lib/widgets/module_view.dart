import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class ModuleView extends StatelessWidget {
  const ModuleView({
    required this.module,
    required this.onPressed,
    this.showReservationStatus = false,
    this.enabled = true,
    this.label,
    super.key,
  });

  final Submodule module;
  final bool showReservationStatus;
  final bool enabled;
  final String? label;
  final VoidCallback? onPressed;

  @override
  Widget build(BuildContext context) {
    final isReserved = module.isReserved();
    return Expanded(
      flex: module.size,
      child: GestureDetector(
        onTap: enabled ? onPressed : null,
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
                      colors: !enabled
                          ? [
                              Colors.white.withOpacity(0.2),
                              Colors.white.withOpacity(0.1),
                            ]
                          : showReservationStatus && !isReserved
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
                        style: TextStyle(
                          height: 0,
                          color: enabled ? Colors.white : Colors.white.withOpacity(0.2),
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
