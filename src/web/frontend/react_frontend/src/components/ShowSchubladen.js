import PropTypes from 'prop-types'
import Orders from "./Orders"


const ShowSchubladen = ({ schubladen: schubladen, onChange }) => {
        return (
                <div>
                        {schubladen ? (
                                <Schubladen
                                        orders={orders}
                                        onChange={onChange}>
                                </Schubladen>
                        ) : (
                                'empty'
                        )}
                </div>
        )
}


ShowSchubladen.protoTypes = {
        schubladen: PropTypes.object,
        onChange: PropTypes.func,
}

export default ShowSchubladen
