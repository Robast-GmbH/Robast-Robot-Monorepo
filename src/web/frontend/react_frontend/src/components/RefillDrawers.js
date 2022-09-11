import { useState } from 'react'
import PropTypes from 'prop-types'
import Button from '@mui/material/Button';
import RefillDrawer from './RefillDrawer';


const RefillDrawers = ({drawers, toggleEmpty}) => {
    return (
            <div class= "refillDrawers">
                    {drawers.map((drawer) => (
                        <RefillDrawer key={drawer.id} drawer = {drawer} toggleEmpty = {toggleEmpty} />
                    ))}
            </div>

            
    )
}

export default RefillDrawers
            