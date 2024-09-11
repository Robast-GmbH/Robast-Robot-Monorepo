import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

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
                    color: enabled ? Colors.black.withOpacity(0.2) : Colors.black.withOpacity(0.05),
                    border: isReserved && enabled
                        ? Border.all(
                            color: Colors.black87,
                            width: 8,
                          )
                        : null,
                    borderRadius: BorderRadius.circular(16),
                  ),
                  child: SizedBox.expand(
                    child: Align(
                      child: Text(
                        label ?? '',
                        textAlign: TextAlign.center,
                        style: TextStyle(
                          height: 0,
                          color: enabled ? RobotColors.secondaryText : RobotColors.primaryText.withOpacity(0.2),
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
