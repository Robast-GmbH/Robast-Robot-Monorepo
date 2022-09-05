import { useState } from 'react'
import PropTypes from 'prop-types'




const AddMapPosition = ({ onAdd }) => {
        const [name, setPositionItemTitle] = useState('')
        const [x, setPositionXCoordinate] = useState(0.0)
        const [y, setPositionYCoordinate] = useState(0.0)

        const onSubmit = (e) => {
                e.preventDefault()
                if (!name) {
                        alert('Add a Title')
                        return
                }
                if (!x) {
                        alert('Add a X-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                if (!y) {
                        alert('Add a Y-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                onAdd({ name, x, y})
                setPositionItemTitle('')
                setPositionXCoordinate(0.0)
                setPositionYCoordinate(0.0)
        }

        return (
                <form className='add-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                                <label>Title</label>
                                <input type='positionItemTitle' placeholder='Kitchen'
                                        value={name} onChange={(e) => setPositionItemTitle(e.target.value)} />
                        </div>

                        <div className='form-control'>
                                <label>X-Coordinate</label>
                                <input type='positionItem' placeholder='0.0'
                                        value={x} onChange={(e) => setPositionXCoordinate(parseFloat(e.target.value))} />
                        </div>

                        <div className='form-control'>
                                <label>Y-Coordinate</label>
                                <input type='positionItem' placeholder='0.0'
                                        value={y} onChange={(e) => setPositionYCoordinate(parseFloat(e.target.value))} />
                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}
AddMapPosition.propTypes = {
        onAdd: PropTypes.func
}
export default AddMapPosition

