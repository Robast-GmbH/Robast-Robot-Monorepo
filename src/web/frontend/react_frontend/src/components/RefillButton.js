
import * as React from 'react';
import PropTypes from 'prop-types'
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ClickAwayListener from '@mui/material/ClickAwayListener';
import Grow from '@mui/material/Grow';
import Paper from '@mui/material/Paper';
import Popper from '@mui/material/Popper';
import MenuItem from '@mui/material/MenuItem';
import MenuList from '@mui/material/MenuList';
import Drawers from './Drawers';

const RefillButton = ({ drawers, refillAction }) => {
    const [open, setOpen] = React.useState(false);
    const anchorRef = React.useRef(null);
    const [selectedIndex, setSelectedIndex] = React.useState(1);
    console.log(drawers[selectedIndex].name)
     if (drawers.length== 0 ){
      return (<div></div>);
     }
        return (
          
            <React.Fragment>
            <ButtonGroup variant="contained" ref={anchorRef} aria-label="split button">
            <Button >{drawers[selectedIndex].name}</Button>{/*onClick={refillAction(drawers[selectedIndex])}*/}
              <Button
                size="small"
                aria-controls={open ? 'split-button-menu' : undefined}
                aria-expanded={open ? 'true' : undefined}
                aria-label="select merge strategy"
                aria-haspopup="menu"
                onClick={setOpen((prevOpen) => !prevOpen)}
              >
              <ArrowDropDownIcon />
              </Button>
            </ButtonGroup>
            <Popper
              sx={{
                zIndex: 1,
              }}
              open={open}
              anchorEl={anchorRef.current}
              role={undefined}
              transition
              disablePortal
            >
              {({ TransitionProps, placement }) => (
                <Grow
                  {...TransitionProps}
                  style={{
                    transformOrigin:
                      placement === 'bottom' ? 'center top' : 'center bottom',
                  }}
                >
                  <Paper>
                    <ClickAwayListener onClickAway={(event)=>{   if (anchorRef.current && anchorRef.current.contains(event.target)) 
                                                            {
                                                                return;
                                                            }
                                                            setOpen(false);}}>

                      <MenuList id="split-button-menu" autoFocusItem>
                        {drawers.map((drawer, index) => (
                          <MenuItem
                            key={drawer.name}
                            disabled={index === 2}
                            selected={index === selectedIndex}
                            onClick={(event) => {
                                                    setSelectedIndex(index);
                                                    setOpen(false);}}
                          >
                            {drawers}
                          </MenuItem>
                        ))}
                      </MenuList>
                    </ClickAwayListener>
                  </Paper>
                </Grow>
              )}
            </Popper>
          </React.Fragment>
        ) 
        
}

export default RefillButton
RefillButton.propTypes = {
        drawers: PropTypes.arrayOf(Drawers),
        refillAction : PropTypes.func,
}


